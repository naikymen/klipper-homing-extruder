# Support for a manual controlled stepper
#
# Copyright (C) 2019-2021  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import stepper, chelper
from . import force_move
import logging

class ManualStepper:
    def __init__(self, config):
        self.printer = config.get_printer()
        if config.get('endstop_pin', None) is not None:
            self.can_home = True
            # NOTE: Instantiate a new PrinterRail class from the
            #       stepper.py module.
            self.rail = stepper.PrinterRail(
                config, need_position_minmax=False, default_position_endstop=0.)
            self.steppers = self.rail.get_steppers()
        else:
            self.can_home = False
            self.rail = stepper.PrinterStepper(config)
            self.steppers = [self.rail]
        self.velocity = config.getfloat('velocity', 5., above=0.)
        self.accel = self.homing_accel = config.getfloat('accel', 0., minval=0.)
        self.next_cmd_time = 0.
        # Setup iterative solver
        ffi_main, ffi_lib = chelper.get_ffi()
        self.trapq = ffi_main.gc(ffi_lib.trapq_alloc(), ffi_lib.trapq_free)
        self.trapq_append = ffi_lib.trapq_append
        self.trapq_finalize_moves = ffi_lib.trapq_finalize_moves
        self.rail.setup_itersolve('cartesian_stepper_alloc', b'x')
        self.rail.set_trapq(self.trapq)

        # NOTE: Additional objects required by "HomingMove.homing_move", needed
        #       to home the manual stepper. See notes below, at "do_homing_move".
        self.event_prefix=""
        self.kinematics={"m": self}

        # Register commands
        stepper_name = config.get_name().split()[1]
        gcode = self.printer.lookup_object('gcode')
        gcode.register_mux_command('MANUAL_STEPPER', "STEPPER",
                                   stepper_name, self.cmd_MANUAL_STEPPER,
                                   desc=self.cmd_MANUAL_STEPPER_help)
    
    def sync_print_time(self):
        """ Synchronize the toolhead's next print time and the local next print time."""
        toolhead = self.printer.lookup_object('toolhead')
        print_time = toolhead.get_last_move_time()
        if self.next_cmd_time > print_time:
            toolhead.dwell(self.next_cmd_time - print_time)
        else:
            self.next_cmd_time = print_time
    
    def do_enable(self, enable):
        self.sync_print_time()
        stepper_enable = self.printer.lookup_object('stepper_enable')
        if enable:
            for s in self.steppers:
                se = stepper_enable.lookup_enable(s.get_name())
                se.motor_enable(self.next_cmd_time)
        else:
            for s in self.steppers:
                se = stepper_enable.lookup_enable(s.get_name())
                se.motor_disable(self.next_cmd_time)
        self.sync_print_time()
    def do_set_position(self, setpos):
        self.rail.set_position([setpos, 0., 0.])
    
    def trapq_append_move(self, print_time, 
                          accel_t=0.0, cruise_t=0.0, decel_t=0.0,
                          start_v=0.0, cruise_v=0.0, accel=0.0,
                          start_pos_x=0.0, axes_r_x=1.0,
                          start_pos_y=0.0, axes_r_y=0.0,
                          start_pos_z=0.0, axes_r_z=0.0):
        self.trapq_append(
            # struct trapq *tq, double print_time
            self.trapq, print_time,
            # double accel_t, double cruise_t, double decel_t
            accel_t, cruise_t, decel_t,
            # double start_pos_x, double start_pos_y, double start_pos_z
            start_pos_x, start_pos_y, start_pos_z,
            # double axes_r_x, double axes_r_y, double axes_r_z
            axes_r_x, axes_r_y, axes_r_z,
            # double start_v, double cruise_v, double accel);
            start_v, cruise_v, accel)

        logging.info(f"\n\ntrapq_append_move: sent move at print_time={print_time}")
    
    def do_move(self, movepos, speed, accel, sync=True):
        self.sync_print_time()
        cp = self.rail.get_commanded_position()
        dist = movepos - cp
        axis_r, accel_t, cruise_t, cruise_v = force_move.calc_move_time(dist, speed, accel)
        
        self.trapq_append_move(
            print_time=self.next_cmd_time,
            accel_t=accel_t, cruise_t=cruise_t, decel_t=accel_t,
            start_pos_x=cp, axes_r_x=axis_r,
            start_v=0.0, cruise_v=cruise_v, accel=accel)
        
        movetime = accel_t + cruise_t + accel_t
        
        # Increment "self.next_cmd_time", call "generate_steps" and "trapq_finalize_moves".
        self.do_final_updates(movetime, sync)

        return movetime
    
    def do_final_updates(self, movetime, sync):
        self.next_cmd_time = self.next_cmd_time + movetime
        # Calls "itersolve_generate_steps" which "Generates 
        # step times for a range of moves on the trapq" (itersolve.c).
        self.rail.generate_steps(self.next_cmd_time)
        # Expire any moves older than `print_time` from the trapezoid velocity queue.
        #   Flush all moves from trapq.
        self.trapq_finalize_moves(self.trapq, self.next_cmd_time + 99999.9,
                                  self.next_cmd_time + 99999.9)
        toolhead = self.printer.lookup_object('toolhead')
        toolhead.note_mcu_movequeue_activity(self.next_cmd_time)
        if sync:
            self.sync_print_time()

    def do_homing_move(self, movepos, speed, accel, triggered, check_trigger):
        if not self.can_home:
            raise self.printer.command_error(
                "No endstop for this manual stepper")
        self.homing_accel = accel
        # NOTE: "movepos" is provided by the user.
        pos = [movepos, 0., 0., 0.]
        # NOTE: this returns "MCU_endstop" objects.
        endstops = self.rail.get_endstops()
        # NOTE: this looks up a "PrinterHoming" object.
        phoming = self.printer.lookup_object('homing')
        # NOTE: "manual_home" is defined in the PrinterHoming class (at homing.py).
        #       The method instantiates a "HomingMove" class by passing it the
        #       "endstops" and "toolhead" objects. Here, the "self" object is passed
        #       as a "virtual toolhead" to "manual_home".
        # NOTE: The requried "endstop"s are from the manual stepper's PrinterRail object.
        # NOTE: "PrinterHoming.manual_home" then calls "HomingMove.homing_move".
        #       The downstream methods in the "HomingMove" class use the
        #       following methods provided by the "virtual toolhead" object 
        #       (i.e. "self", this very class):
        #       - flush_step_generation
        #       - get_kinematics:           returning a "kin" object with methods:
        #           - kin.get_steppers:     returning a list of stepper objects.
        #           - kin.calc_position:    returning ???
        #       - get_position:             returning "thpos" (toolhead position)
        #       - get_last_move_time:       returning "print_time" (and later "move_end_print_time")
        #       - dwell
        #       - drip_move
        #       - set_position
        # NOTE: Other methods using the toolhead object or derivatives are also called:
        #       -   calc_toolhead_pos: This method receives a "movepos" argument,
        #           which is the "pos" list above:  pos = [0., 0., 0., 0.]
        # NOTE: Of these methods, the Extruder class defines none.
        # NOTE: The object returned by "get_kinematics" is
        #       required to have the following methods:
        #       - get_steppers()
        #       - calc_position(kin_spos)
        # NOTE: The following command ends up calling the methods 
        #       in this class. For example "drip_move" for moving
        #       the extruder (towards the endstop, ideally).
        # NOTE: There are also other methods for homing:
        #       - probing_move ???
        #       - cmd_G28: ???
        # TODO: Consider using those alternative methods.
        phoming.manual_home(toolhead=self, endstops=endstops, pos=pos, speed=speed,
                            triggered=triggered, check_triggered=check_trigger)
    
    cmd_MANUAL_STEPPER_help = "Command a manually configured stepper"
    def cmd_MANUAL_STEPPER(self, gcmd):
        """
        Usage: MANUAL_STEPPER STEPPER=config_name [ENABLE=[0|1]] [SET_POSITION=<pos>] [SPEED=<speed>] [ACCEL=<accel>] [MOVE=<pos> [STOP_ON_ENDSTOP=[1|2|-1|-2]] [SYNC=0]]

        Homing example: MANUAL_STEPPER STEPPER=manolo MOVE=100 STOP_ON_ENDSTOP=1

        If STOP_ON_ENDSTOP=1 is specified then the move will end early should the endstop report as triggered.
        Use STOP_ON_ENDSTOP=2 to complete the move without error even if the endstop does not trigger, 
        and use -1 or -2 to stop when the endstop reports not triggered. This is similar to LinuxCNC's G38.n commands.
        """
        # Enabling
        enable = gcmd.get_int('ENABLE', None)
        if enable is not None:
            self.do_enable(enable)
        # Setting position
        setpos = gcmd.get_float('SET_POSITION', None)
        if setpos is not None:
            self.do_set_position(setpos)
        # Parameters
        speed = gcmd.get_float('SPEED', self.velocity, above=0.)
        accel = gcmd.get_float('ACCEL', self.accel, minval=0.)
        homing_move = gcmd.get_int('STOP_ON_ENDSTOP', 0)
        
        # Route the commmand to the corresponding implementation.
        # Homing.
        if homing_move:
            movepos = gcmd.get_float('MOVE')
            self.do_homing_move(movepos, speed, accel,
                                homing_move > 0, abs(homing_move) == 1)
        # Regular move.
        elif gcmd.get_float('MOVE', None) is not None:
            movepos = gcmd.get_float('MOVE')
            # NOTE: "Normally future G-Code commands will be scheduled to run 
            #       after the stepper move completes, however if a manual
            #       stepper move uses SYNC=0 then future G-Code movement 
            #       commands may run in parallel with the stepper movement."
            sync = gcmd.get_int('SYNC', 1)
            self.do_move(movepos, speed, accel, sync)
        # No move.
        elif gcmd.get_int('SYNC', 0):
            self.sync_print_time()
    
    # Toolhead wrappers to support homing
    def flush_step_generation(self):
        # NOTE: this is the first function called by "homing_move",
        #       before "noting start location". It is also called
        #       bedore "determining the stepper halt positions".
        # NOTE: It either sets "self.next_cmd_time" to the value
        #       of "toolhead.get_last_move_time" or sends a dwell 
        #       command.
        self.sync_print_time()
    def get_position(self):
        return [self.rail.get_commanded_position(), 0., 0., 0.]
    def set_position(self, newpos, homing_axes=()):
        self.do_set_position(newpos[0])
    def get_last_move_time(self):
        self.sync_print_time()
        return self.next_cmd_time
    def dwell(self, delay):
        self.next_cmd_time += max(0., delay)
    def drip_move(self, newpos, speed, drip_completion):
        self.do_move(newpos[0], speed, self.homing_accel)
    def get_kinematics(self):
        return self
    def get_steppers(self):
        # NOTE: In homing.py, the "steppers" list is parsed
        #       calling "get_name" and "get_step_dist" on each.
        # NOTE: Extruder steppers donot have these methods,
        #       but they have a proper stepper definition internally,
        #       which can be accesed with "extruder_stepper.stepper".
        return self.steppers
    def calc_position(self, stepper_positions):
        return [stepper_positions[self.rail.get_name()], 0., 0.]

def load_config_prefix(config):
    return ManualStepper(config)
