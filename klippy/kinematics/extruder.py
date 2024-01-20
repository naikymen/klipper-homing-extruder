# Code for handling printer nozzle extruders
#
# Copyright (C) 2016-2022  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math, logging
import stepper, chelper

class ExtruderStepper:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        self.pressure_advance = self.pressure_advance_smooth_time = 0.
        self.config_pa = config.getfloat('pressure_advance', 0., minval=0.)
        self.config_smooth_time = config.getfloat(
                'pressure_advance_smooth_time', 0.040, above=0., maxval=.200)
        
        # Setup stepper
        # NOTE: In the manual_stepper class, the "rail" is defined
        #       either from PrinterRail or PrinterStepper. The first
        #       is used when an endstop pin was configured.
        if config.get('endstop_pin', None) is not None:
            # NOTE: Setup home-able extruder steppers.
            self.can_home = True
            self.rail = stepper.PrinterRail(config)     # PrinterRail
            # NOTE: "rail.get_steppers" returns a list of PrinterStepper (MCU_stepper) objects.
            self.steppers = self.rail.get_steppers()    # [MCU_stepper]
        else:
            # NOTE: Setup "regular" unhome-able extruder steppers.
            self.can_home = False
            # NOTE: "PrinterStepper" returns an MCU_stepper object.
            self.rail = stepper.PrinterStepper(config)  # MCU_stepper
            self.steppers = [self.rail]                 # [MCU_stepper]
        # NOTE: "steppers" from PrinterRail are interanlly defined from PrinterStepper,
        #       and thus should be equivalent. An exception is the "get_steppers" 
        #       method, which the MCU_stepper object returned "PrinterStepper" does
        #       not have, but the PrinterRail does. This has been patched.
        self.stepper = self.steppers[0]

        # NOTE: Setup attributes for limit checks, useful for syringe extruders.
        # TODO: Check if this works as expected for extruder limit checking.
        self.limits = [(1.0, -1.0)]
        # NOTE: These values are replaced by "self._handle_connect" 
        #       if the stepper can be homed.
        self.axes_min, self.axes_max = None, None

        # Register a handler for turning off the steppers.
        self.printer.register_event_handler("stepper_enable:motor_off",
                                            self._motor_off)
        
        ffi_main, ffi_lib = chelper.get_ffi()
        self.sk_extruder = ffi_main.gc(ffi_lib.extruder_stepper_alloc(),
                                       ffi_lib.free)
        self.stepper.set_stepper_kinematics(self.sk_extruder)
        self.motion_queue = None
        # Register commands
        self.printer.register_event_handler("klippy:connect",
                                            self._handle_connect)
        gcode = self.printer.lookup_object('gcode')
        if self.name == 'extruder':
            gcode.register_mux_command("SET_PRESSURE_ADVANCE", "EXTRUDER", None,
                                       self.cmd_default_SET_PRESSURE_ADVANCE,
                                       desc=self.cmd_SET_PRESSURE_ADVANCE_help)
        gcode.register_mux_command("SET_PRESSURE_ADVANCE", "EXTRUDER",
                                   self.name, self.cmd_SET_PRESSURE_ADVANCE,
                                   desc=self.cmd_SET_PRESSURE_ADVANCE_help)
        gcode.register_mux_command("SET_EXTRUDER_ROTATION_DISTANCE", "EXTRUDER",
                                   self.name, self.cmd_SET_E_ROTATION_DISTANCE,
                                   desc=self.cmd_SET_E_ROTATION_DISTANCE_help)
        gcode.register_mux_command("SYNC_EXTRUDER_MOTION", "EXTRUDER",
                                   self.name, self.cmd_SYNC_EXTRUDER_MOTION,
                                   desc=self.cmd_SYNC_EXTRUDER_MOTION_help)
        gcode.register_mux_command("SET_EXTRUDER_STEP_DISTANCE", "EXTRUDER",
                                   self.name, self.cmd_SET_E_STEP_DISTANCE,
                                   desc=self.cmd_SET_E_STEP_DISTANCE_help)
        gcode.register_mux_command("SYNC_STEPPER_TO_EXTRUDER", "STEPPER",
                                   self.name, self.cmd_SYNC_STEPPER_TO_EXTRUDER,
                                   desc=self.cmd_SYNC_STEPPER_TO_EXTRUDER_help)
    def _handle_connect(self):
        toolhead = self.printer.lookup_object('toolhead')
        toolhead.register_step_generator(self.stepper.generate_steps)
        self._set_pressure_advance(self.config_pa, self.config_smooth_time)

        # NOTE: Setup attributes for limit checks, useful for syringe extruders.
        if self.can_home:
            range = self.rail.get_range()
            self.axes_min = toolhead.Coord(e=range[0])
            self.axes_max = toolhead.Coord(e=range[1])

    def _motor_off(self, print_time):
        # Handler for "turning off" the steppers (borrowed from "cartesian").
        # NOTE: The effect is that move checks will never pass, and an error
        #       indicating "homing is needed" will end up being raised.
        self.limits = [(1.0, -1.0)]
    
    def get_status(self, eventtime):

        status = {'pressure_advance': self.pressure_advance,
                  'smooth_time': self.pressure_advance_smooth_time,
                  'motion_queue': self.motion_queue}

        status.update(self.get_limit_status(eventtime))

        return status

    def get_limit_status(self, eventtime, prev=None):
        # Default output.
        status=dict()
        
        # TODO: Clean this up! This code is duplicated in several places,
        #       At least in the extruder stepper and in the xxx_abc.py kinematics.
        
        # Only enter here if the extruder stepper is home-able.
        if self.can_home:
            # Compute the homed axes.
            homed_axes = [a for a, (l, h) in zip("e", self.limits) if l <= h]
            homed_axes = "".join(homed_axes)
            # This is the regular output, when no "previous" info must be updated.
            if prev is None:
                status = {
                    'homed_axes': homed_axes,
                    'axis_minimum': self.axes_min,
                    'axis_maximum': self.axes_max,
                }
            else:
                # If there is previous info to update, 
                # handle the cases one by one.
                
                # Concatenate homed axes.
                if 'homed_axes' in prev.keys():
                    prev['homed_axes'] += homed_axes
                else:
                    prev['homed_axes'] = homed_axes
                
                # Update minimum limits.
                if 'axis_minimum' in prev.keys():
                    value = getattr(self.axes_min, "e")
                    prev['axis_minimum'] = prev['axis_minimum']._replace(e=value)
                else:
                    prev['axis_minimum'] = self.axes_min

                # Update maximum limits.
                if 'axis_maximum' in prev.keys():
                    value = getattr(self.axes_max, "e")
                    prev['axis_maximum'] = prev['axis_maximum']._replace(e=value)
                else:
                    prev['axis_maximum'] = self.axes_max
                
                status = prev

        # Done!
        return status
    
    def find_past_position(self, print_time):
        mcu_pos = self.stepper.get_past_mcu_position(print_time)
        return self.stepper.mcu_to_commanded_position(mcu_pos)
    
    def sync_to_extruder(self, extruder_name):
        # NOTE: from the following I guess that the
        #       "SYNC_STEPPER_TO_EXTRUDER" and "SYNC_EXTRUDER_MOTION"
        #       commands mainly swap the "trapq" movement queues,
        #       using the "set_trapq" method from stepper.py
        toolhead = self.printer.lookup_object('toolhead')
        toolhead.flush_step_generation()
        if not extruder_name:
            self.stepper.set_trapq(None)
            self.motion_queue = None
            return
        extruder = self.printer.lookup_object(extruder_name, None)
        if extruder is None or not isinstance(extruder, PrinterExtruder):
            raise self.printer.command_error("'%s' is not a valid extruder."
                                             % (extruder_name,))
        self.stepper.set_position([extruder.last_position, 0., 0.])
        self.stepper.set_trapq(extruder.get_trapq())
        self.motion_queue = extruder_name

    def check_move_limits(self, move):
        """ExtruderStepper version of check_move_limits in toolhead.py"""
        epos = move.end_pos[-1]

        if self.can_home:
            # NOTE: Software limit checks, borrowed from "cartesian.py".
            logging.info("\n\n" + f"extruder_stepper.check_move_limits: checking move ending on epos={epos}" + "\n\n")
            if (epos < self.limits[0][0] or epos > self.limits[0][1]):
                self._check_endstops(move)
        else:
            logging.info("\n\n" + f"extruder_stepper.check_move_limits: E stepper not home-able, skipping check on move ending on epos={epos}" + "\n\n")
        
    def _check_endstops(self, move):
        """ExtruderStepper version of _check_endstops in toolhead.py"""

        # NOTE: Software limit checks, borrowed from "cartesian.py".
        logging.info("\n\n" + f"extruder_stepper._check_endstops: move limit check triggered.\n\n")
        end_pos = move.end_pos[-1]
        
        # NOTE: Check if the extruder move is out of bounds.
        if (move.axes_d[-1] and (end_pos < self.limits[0][0] or end_pos > self.limits[0][1])):
            # NOTE: The move is not allowed, check if this is due to unhomed axis.
            if self.limits[0][0] > self.limits[0][1]:
                # NOTE: "self.limits" will be "(1.0, -1.0)" when not homed, triggering this.
                logging.info(f"extruder._check_endstops: Must home extruder axis ({len(move.end_pos)}) first.")
                raise move.move_error(f"Must home extruder axis ({len(move.end_pos)}) first.")
            # NOTE: Else raise a move error without a message.
            raise move.move_error()
        else:
            # NOTE: Everything seems fine.
            logging.info(f"extruder_stepper._check_endstops: The extruder's move to {end_pos} on axis {len(move.end_pos)} checks out.")
    
    def set_position(self, newpos_e, homing_e=False, print_time=None):
        """ExtruderStepper version of set_position in toolhead.py"""
        logging.info("\n\n" + f"ExtruderStepper.set_position: setting E to newpos={newpos_e}.\n\n")

        # NOTE: The following calls PrinterRail.set_position, which
        #       calls set_position on each of the MCU_stepper objects 
        #       in each PrinterRail.
        #       It eventually calls "itersolve_set_position".
        self.rail.set_position([newpos_e, 0., 0.])

        # TODO: the "homing_axes" parameter is not used rait nau.
        # NOTE: Set limits if the axis is (being) homed.
        if homing_e and self.can_home:
            # NOTE: This will put the axis to a "homed" state, which means that
            #       the unhomed part of the kinematic move check will pass from
            #       now on.
            logging.info(f"\n\nExtruderStepper: setting limits={self.rail.get_range()} on stepper: {self.rail.get_name()}\n\n")
            self.limits[0] = self.rail.get_range()
    
    def _set_pressure_advance(self, pressure_advance, smooth_time):
        old_smooth_time = self.pressure_advance_smooth_time
        if not self.pressure_advance:
            old_smooth_time = 0.
        new_smooth_time = smooth_time
        if not pressure_advance:
            new_smooth_time = 0.
        toolhead = self.printer.lookup_object("toolhead")
        toolhead.note_step_generation_scan_time(new_smooth_time * .5,
                                                old_delay=old_smooth_time * .5)
        ffi_main, ffi_lib = chelper.get_ffi()
        espa = ffi_lib.extruder_set_pressure_advance
        espa(self.sk_extruder, pressure_advance, new_smooth_time)
        self.pressure_advance = pressure_advance
        self.pressure_advance_smooth_time = smooth_time
    cmd_SET_PRESSURE_ADVANCE_help = "Set pressure advance parameters"
    def cmd_default_SET_PRESSURE_ADVANCE(self, gcmd):
        extruder = self.printer.lookup_object('toolhead').get_extruder()
        if extruder.extruder_stepper is None:
            raise gcmd.error("Active extruder does not have a stepper")
        strapq = extruder.extruder_stepper.stepper.get_trapq()
        if strapq is not extruder.get_trapq():
            raise gcmd.error("Unable to infer active extruder stepper")
        extruder.extruder_stepper.cmd_SET_PRESSURE_ADVANCE(gcmd)
    def cmd_SET_PRESSURE_ADVANCE(self, gcmd):
        pressure_advance = gcmd.get_float('ADVANCE', self.pressure_advance,
                                          minval=0.)
        smooth_time = gcmd.get_float('SMOOTH_TIME',
                                     self.pressure_advance_smooth_time,
                                     minval=0., maxval=.200)
        self._set_pressure_advance(pressure_advance, smooth_time)
        msg = ("pressure_advance: %.6f\n"
               "pressure_advance_smooth_time: %.6f"
               % (pressure_advance, smooth_time))
        self.printer.set_rollover_info(self.name, "%s: %s" % (self.name, msg))
        gcmd.respond_info(msg, log=False)
    cmd_SET_E_ROTATION_DISTANCE_help = "Set extruder rotation distance"
    def cmd_SET_E_ROTATION_DISTANCE(self, gcmd):
        rotation_dist = gcmd.get_float('DISTANCE', None)
        if rotation_dist is not None:
            if not rotation_dist:
                raise gcmd.error("Rotation distance can not be zero")
            invert_dir, orig_invert_dir = self.stepper.get_dir_inverted()
            next_invert_dir = orig_invert_dir
            if rotation_dist < 0.:
                next_invert_dir = not orig_invert_dir
                rotation_dist = -rotation_dist
            toolhead = self.printer.lookup_object('toolhead')
            toolhead.flush_step_generation()
            self.stepper.set_rotation_distance(rotation_dist)
            self.stepper.set_dir_inverted(next_invert_dir)
        else:
            rotation_dist, spr = self.stepper.get_rotation_distance()
        invert_dir, orig_invert_dir = self.stepper.get_dir_inverted()
        if invert_dir != orig_invert_dir:
            rotation_dist = -rotation_dist
        gcmd.respond_info("Extruder '%s' rotation distance set to %0.6f"
                          % (self.name, rotation_dist))
    cmd_SYNC_EXTRUDER_MOTION_help = "Set extruder stepper motion queue"
    def cmd_SYNC_EXTRUDER_MOTION(self, gcmd):
        """
        This command will cause the stepper specified by EXTRUDER (as defined in 
        an [extruder] or [extruder_stepper] config section) to become synchronized 
        to the movement of an extruder specified by MOTION_QUEUE (as defined in an 
        [extruder] config section).
        If MOTION_QUEUE is an empty string then the stepper will be desynchronized 
        from all extruder movement.
        
        Usage: SYNC_EXTRUDER_MOTION EXTRUDER=<name> MOTION_QUEUE=<name>
        """
        ename = gcmd.get('MOTION_QUEUE')
        self.sync_to_extruder(ename)
        gcmd.respond_info("Extruder '%s' now syncing with '%s'"
                          % (self.name, ename))
    cmd_SET_E_STEP_DISTANCE_help = "Set extruder step distance"
    def cmd_SET_E_STEP_DISTANCE(self, gcmd):
        step_dist = gcmd.get_float('DISTANCE', None, above=0.)
        if step_dist is not None:
            toolhead = self.printer.lookup_object('toolhead')
            toolhead.flush_step_generation()
            rd, steps_per_rotation = self.stepper.get_rotation_distance()
            self.stepper.set_rotation_distance(step_dist * steps_per_rotation)
        else:
            step_dist = self.stepper.get_step_dist()
        gcmd.respond_info("Extruder '%s' step distance set to %0.6f"
                          % (self.name, step_dist))
    cmd_SYNC_STEPPER_TO_EXTRUDER_help = "Set extruder stepper"
    # NOTE: From the docs "This command is deprecated and will be removed in the near future."
    def cmd_SYNC_STEPPER_TO_EXTRUDER(self, gcmd):
        ename = gcmd.get('EXTRUDER')
        self.sync_to_extruder(ename)
        gcmd.respond_info("Extruder '%s' now syncing with '%s'"
                          % (self.name, ename))

# Tracking for hotend heater, extrusion motion queue, and extruder stepper
class PrinterExtruder:
    # NOTE: this class is instantiated once per [extruder] section in the config.
    def __init__(self, config, extruder_num):
        self.printer = config.get_printer()
        self.name = config.get_name()
        self.last_position = 0.
        
        # Setup hotend heater
        shared_heater = config.get('shared_heater', None)
        pheaters = self.printer.load_object(config, 'heaters')
        gcode_id = 'T%d' % (extruder_num,)
        if shared_heater is None:
            self.heater = pheaters.setup_heater(config, gcode_id)
        else:
            config.deprecate('shared_heater')
            self.heater = pheaters.lookup_heater(shared_heater)
        
        # Setup kinematic checks
        self.nozzle_diameter = config.getfloat('nozzle_diameter', above=0.)
        filament_diameter = config.getfloat(
            'filament_diameter', minval=self.nozzle_diameter)
        self.filament_area = math.pi * (filament_diameter * .5)**2
        def_max_cross_section = 4. * self.nozzle_diameter**2
        def_max_extrude_ratio = def_max_cross_section / self.filament_area
        max_cross_section = config.getfloat(
            'max_extrude_cross_section', def_max_cross_section, above=0.)
        self.max_extrude_ratio = max_cross_section / self.filament_area
        logging.info("Extruder max_extrude_ratio=%.6f", self.max_extrude_ratio)
        toolhead = self.printer.lookup_object('toolhead')
        max_velocity, max_accel = toolhead.get_max_velocity()
        self.max_e_velocity = config.getfloat(
            'max_extrude_only_velocity', max_velocity * def_max_extrude_ratio
            , above=0.)
        self.max_e_accel = config.getfloat(
            'max_extrude_only_accel', max_accel * def_max_extrude_ratio
            , above=0.)
        self.max_e_dist = config.getfloat(
            'max_extrude_only_distance', 50., minval=0.)
        self.instant_corner_v = config.getfloat(
            'instantaneous_corner_velocity', 1., minval=0.)

        # NOTE: Get the axis ID (index) of the extruder axis. Will
        #       be equal to the amount of axes in the toolhead,
        #       either XYZ=3 or XYZABC=6.
        self.axis_idx = toolhead.axis_count
        
        # Setup extruder trapq (trapezoidal motion queue)
        ffi_main, ffi_lib = chelper.get_ffi()
        self.trapq = ffi_main.gc(ffi_lib.trapq_alloc(), ffi_lib.trapq_free)
        self.trapq_append = ffi_lib.trapq_append
        self.trapq_finalize_moves = ffi_lib.trapq_finalize_moves
        self.trapq_set_position = ffi_lib.trapq_set_position
        
        # Setup extruder stepper
        # NOTE: an ExtruderStepper class is instantiated if no pins
        #       were defined in the extruder config section (step/dir/...).
        self.extruder_stepper = None
        if (config.get('step_pin', None) is not None
            or config.get('dir_pin', None) is not None
            or config.get('rotation_distance', None) is not None):
            # TODO: it might be possible to add an endstop after this.
            self.extruder_stepper = ExtruderStepper(config)
            self.extruder_stepper.stepper.set_trapq(self.trapq)
        
        # Register commands
        gcode = self.printer.lookup_object('gcode')
        if self.name == 'extruder':
            toolhead.set_extruder(self, 0.)
            gcode.register_command("M104", self.cmd_M104)
            gcode.register_command("M109", self.cmd_M109)
        # NOTE: a mux command is registered and identified uniquely by the "cmd",
        #       the "key", and also the key's "value". This means that the 
        #       ACTIVATE_EXTRUDER command will run in different instances
        #       of PrinterExtruder classes if the "value" differs.
        gcode.register_mux_command(cmd="ACTIVATE_EXTRUDER", key="EXTRUDER",
                                   value=self.name, func=self.cmd_ACTIVATE_EXTRUDER,
                                   desc=self.cmd_ACTIVATE_EXTRUDER_help)
    def update_move_time(self, flush_time):
        # NOTE: "Expire any moves older than `flush_time` from the trapezoid velocity queue"
        self.trapq_finalize_moves(self.trapq, flush_time)
    def get_status(self, eventtime):
        sts = self.heater.get_status(eventtime)
        sts['can_extrude'] = self.heater.can_extrude
        if self.extruder_stepper is not None:
            sts.update(self.extruder_stepper.get_status(eventtime))
        return sts
    def get_name(self):
        return self.name
    def get_heater(self):
        return self.heater
    def get_trapq(self):
        return self.trapq
    def stats(self, eventtime):
        return self.heater.stats(eventtime)
    
    def check_move(self, move):
        # NOTE: get the extruder component of the move (ratio of total displacement).
        axis_r = move.axes_r[-1]
        
        # NOTE: error-out if the extruder is not ready (not hot enough).
        if not self.heater.can_extrude:
            raise self.printer.command_error(
                "Extrude below minimum temp\n"
                "See the 'min_extrude_temp' config option for details")
        
        # NOTE: other extrusion checks.
        if (not move.axes_d[0] and not move.axes_d[1]) or axis_r < 0.:
            # Extrude only move (or retraction move) - limit accel and velocity
            if abs(move.axes_d[-1]) > self.max_e_dist:
                raise self.printer.command_error(
                    "Extrude only move too long (%.3fmm vs %.3fmm)\n"
                    "See the 'max_extrude_only_distance' config"
                    " option for details" % (move.axes_d[-1], self.max_e_dist))
            inv_extrude_r = 1. / abs(axis_r)
            move.limit_speed(self.max_e_velocity * inv_extrude_r,
                             self.max_e_accel * inv_extrude_r)
        elif axis_r > self.max_extrude_ratio:
            if move.axes_d[-1] <= self.nozzle_diameter * self.max_extrude_ratio:
                # Permit extrusion if amount extruded is tiny
                return
            area = axis_r * self.filament_area
            logging.debug("Overextrude: %s vs %s (area=%.3f dist=%.3f)",
                          axis_r, self.max_extrude_ratio, area, move.move_d)
            raise self.printer.command_error(
                "Move exceeds maximum extrusion (%.3fmm^2 vs %.3fmm^2)\n"
                "See the 'max_extrude_cross_section' config option for details"
                % (area, self.max_extrude_ratio * self.filament_area))
        
        # NOTE: implement software limit checks.
        self.extruder_stepper.check_move_limits(move)

    def set_position(self, newpos_e, homing_axes=(), print_time=None):
        """PrinterExtruder version of set_position in 'toolhead.py', 
        called by its 'set_position_e' method.
        Should set the position in the 'trapq' and in the 'extruder kin'.
        """
        if print_time is None:
            toolhead = self.printer.lookup_object('toolhead')
            print_time = toolhead.print_time

        logging.info(f"\n\nPrinterExtruder.set_position: called with newpos_e={newpos_e} homing_axes={homing_axes} and self.axis_idx={self.axis_idx}.\n\n")

        # Set the TRAPQ's position
        self.trapq_set_position(self.trapq, print_time, newpos_e, 0., 0.)
        
        # NOTE: Check if the E axis is being homed. This
        #       will signal the stepper to set its limits 
        #       and appear as "homed".
        # NOTE: 'homing_axes' should contain a value equal to 'self.toolhead.pos_length'
        #       in this case (e.g. '4' in an XYZE setup). See 'cmd_HOME_EXTRUDER' at 'extruder_home.py'.
        homing_e = self.axis_idx in homing_axes

        # NOTE: Have the ExtruderStepper set its "MCU_stepper" position.
        self.extruder_stepper.set_position(newpos_e=newpos_e,
                                           homing_e=homing_e,
                                           print_time=print_time)


    def calc_junction(self, prev_move, move):
        diff_r = move.axes_r[-1] - prev_move.axes_r[-1]
        if diff_r:
            return (self.instant_corner_v / abs(diff_r))**2
        return move.max_cruise_v2
    def move(self, print_time, move):
        # NOTE: this PrinterExtruder.move method is called
        #       by the _process_moves method from ToolHead.
        #       In that call, the "print_time" is shared with
        #       the main XYZ stepper queue (sent to "trapq"),
        #       which is probably responsible for the synced
        #       motion of the extruder stepper and the XYZ 
        #       axes.
        # NOTE: the "move" argument comes from a list of moves.
        #       A "move" is appended to that list by calls to the "ToolHead.add_move" method.
        #       The "add_move" method is called by the "ToolHead.move" method.
        #       which creates the "move" object by instantiating a Move class,
        #       with the following arguments:
        #       - toolhead=self:                 the ToolHead class itself.
        #       - start_pos=self.commanded_pos:  list of "initial" coordinates [0.0, 0.0, 0.0, 0.0]
        #       - end_pos=newpos:                ???
        #       - speed=speed:                   ???
        axis_r = move.axes_r[-1]
        accel = move.accel * axis_r
        start_v = move.start_v * axis_r
        cruise_v = move.cruise_v * axis_r
        can_pressure_advance = False
        if axis_r > 0. and (move.axes_d[0] or move.axes_d[1] or move.axes_d[3] or move.axes_d[4] or move.axes_d[5]):
            can_pressure_advance = True
        # Queue movement (x is extruder movement, y is pressure advance flag)
        # NOTE: the following "self.trapq" was setup during this class's init.
        # TODO: after reading the code overview (https://www.klipper3d.org/Code_Overview.html)
        #       I still don't know for sure _where_ in the code these queues of 
        #       moves end up together. The only reasonable place left seems to be
        #       in the "serialqueue.c" or nearby files. What I know is that they 
        #       are coordinated by print_time at "_process_moves" (see: toolhead.py).
        self.trapq_append(self.trapq, print_time,
                          move.accel_t, move.cruise_t, move.decel_t,
                          move.start_pos[-1], 0., 0.,
                          1., can_pressure_advance, 0.,
                          start_v, cruise_v, accel)
        self.last_position = move.end_pos[-1]
        logging.info(f"\n\nextruder: move.end_pos[-1]={str(move.end_pos[-1])}\n\n")
    def find_past_position(self, print_time):
        if self.extruder_stepper is None:
            return 0.
        return self.extruder_stepper.find_past_position(print_time)
    
    def calc_position(self, stepper_positions):
        # NOTE: borrowed from the cartesian kinematics for "homing.py".
        return [stepper_positions[rail.get_name()] for rail in self.rails]
    
    def cmd_M104(self, gcmd, wait=False):
        # Set Extruder Temperature
        temp = gcmd.get_float('S', 0.)
        index = gcmd.get_int('T', None, minval=0)
        if index is not None:
            section = 'extruder'
            if index:
                section = 'extruder%d' % (index,)
            extruder = self.printer.lookup_object(section, None)
            if extruder is None:
                if temp <= 0.:
                    return
                raise gcmd.error("Extruder not configured")
        else:
            extruder = self.printer.lookup_object('toolhead').get_extruder()
        pheaters = self.printer.lookup_object('heaters')
        pheaters.set_temperature(extruder.get_heater(), temp, wait)
    def cmd_M109(self, gcmd):
        # Set Extruder Temperature and Wait
        self.cmd_M104(gcmd, wait=True)
    cmd_ACTIVATE_EXTRUDER_help = "Change the active extruder"
    def cmd_ACTIVATE_EXTRUDER(self, gcmd):
        toolhead = self.printer.lookup_object('toolhead')
        if toolhead.get_extruder() is self:
            gcmd.respond_info("Extruder %s already active" % (self.name,))
            return
        gcmd.respond_info("Activating extruder %s" % (self.name,))
        toolhead.flush_step_generation()
        # NOTE: the following "set_extruder" method replaces the extruder
        #       object in the toolhead with "self" (this extruder instance),
        #       and replaces the fourth coordinate of the toolhead's "commanded
        #       position" with the "last position" of this extruder.
        toolhead.set_extruder(extruder=self, extrude_pos=self.last_position)
        # NOTE: the following triggers the "_handle_activate_extruder" method
        #       in "gcode_move.py".
        self.printer.send_event("extruder:activate_extruder")

# Dummy extruder class used when a printer has no extruder at all
# NOTE: this dummy extruder class is used to initialize 
#       ToolHead classes at ToolHead.py.
class DummyExtruder:
    name = None
    def __init__(self, printer):
        self.printer = printer
    def update_move_time(self, flush_time):
        pass
    def check_move(self, move):
        raise move.move_error("Extrude when no extruder present")
    def find_past_position(self, print_time):
        return 0.
    def calc_junction(self, prev_move, move):
        return move.max_cruise_v2
    def get_name(self):
        return self.name
    def get_heater(self):
        raise self.printer.command_error("Extruder not configured")
    def get_trapq(self):
        raise self.printer.command_error("Extruder not configured")

def add_printer_objects(config):
    printer = config.get_printer()
    for i in range(99):
        section = 'extruder'
        if i:
            section = 'extruder%d' % (i,)
        if not config.has_section(section):
            break
        pe = PrinterExtruder(config.getsection(section), i)
        printer.add_object(section, pe)
