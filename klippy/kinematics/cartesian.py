# Code for handling the kinematics of cartesian robots
#
# Copyright (C) 2016-2021  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
import stepper
from . import idex_modes

class CartKinematics:
    def __init__(self, toolhead, config, trapq=None):
        
        # Axis names
        self.axis = [0, 1, 2]
        self.axis_names = "".join([toolhead.axis_names[i] for i in self.axis])  # Will get "XYZ" from "XYZABC"
        self.toolhead_axis_count = toolhead.axis_count  # len(self.axis_names)

        logging.info(f"CartKinematics: starting setup with axes: {self.axis_names}")
        
        # Get the trapq
        if trapq is None:
            self.trapq = toolhead.get_trapq()
        else:
            self.trapq = trapq
        
        self.printer = config.get_printer()
        # Setup axis rails
        self.dual_carriage_axis = None
        self.dual_carriage_rails = []
        # NOTE: a "PrinterRail" is setup by LookupMultiRail, per each 
        #       of the three axis, including their corresponding endstops.
        # NOTE: The "self.rails" list contains "PrinterRail" objects, which
        #       can have one or more stepper (PrinterStepper/MCU_stepper) objects.
        self.rails = [stepper.LookupMultiRail(config.getsection('stepper_' + n))
                      for n in 'xyz']
        for rail, axis in zip(self.rails, 'xyz'):
            rail.setup_itersolve('cartesian_stepper_alloc', axis.encode())
        ranges = [r.get_range() for r in self.rails]
        self.axes_min = toolhead.Coord(*[r[0] for r in ranges], e=0.)
        self.axes_max = toolhead.Coord(*[r[1] for r in ranges], e=0.)
        self.dc_module = None
        if config.has_section('dual_carriage'):
            dc_config = config.getsection('dual_carriage')
            dc_axis = dc_config.getchoice('axis', {'x': 'x', 'y': 'y'})
            self.dual_carriage_axis = {'x': 0, 'y': 1}[dc_axis]
            # setup second dual carriage rail
            self.rails.append(stepper.LookupMultiRail(dc_config))
            self.rails[3].setup_itersolve('cartesian_stepper_alloc',
                                          dc_axis.encode())
            dc_rail_0 = idex_modes.DualCarriagesRail(
                    self.rails[self.dual_carriage_axis],
                    axis=self.dual_carriage_axis, active=True)
            dc_rail_1 = idex_modes.DualCarriagesRail(
                    self.rails[3], axis=self.dual_carriage_axis, active=False)
            self.dc_module = idex_modes.DualCarriages(
                    dc_config, dc_rail_0, dc_rail_1,
                    axis=self.dual_carriage_axis)
        for s in self.get_steppers():
            s.set_trapq(self.trapq)
            toolhead.register_step_generator(s.generate_steps)
        self.printer.register_event_handler("stepper_enable:motor_off",
                                            self._motor_off)
        # Setup boundary checks
        max_velocity, max_accel = toolhead.get_max_velocity()
        self.max_z_velocity = config.getfloat('max_z_velocity', max_velocity,
                                              above=0., maxval=max_velocity)
        self.max_z_accel = config.getfloat('max_z_accel', max_accel,
                                           above=0., maxval=max_accel)
        self.limits = [(1.0, -1.0)] * 3
    def get_steppers(self):
        # NOTE: The "self.rails" list contains "PrinterRail" objects, which
        #       can have one or more stepper (PrinterStepper/MCU_stepper) objects.
        # NOTE: run "get_steppers" on each "PrinterRail" object from 
        #       the "self.rails" list. That method returns the list of
        #       all "PrinterStepper"/"MCU_stepper" objects in the kinematic.
        return [s for rail in self.rails for s in rail.get_steppers()]
    def calc_position(self, stepper_positions):
        return [stepper_positions[rail.get_name()] for rail in self.rails]
    def update_limits(self, i, range):
        l, h = self.limits[i]
        # Only update limits if this axis was already homed,
        # otherwise leave in un-homed state.
        if l <= h:
            self.limits[i] = range
    def override_rail(self, i, rail):
        self.rails[i] = rail
    def set_position(self, newpos, homing_axes):
        logging.info(f"CartKinematics.set_position: setting kinematic position of {len(self.rails)} rails " +
                     f"with newpos={newpos} and homing_axes={homing_axes}")
        for i, rail in enumerate(self.rails):
            logging.info(f"CartKinematics: setting newpos={newpos} on stepper: {rail.get_name()}")
            # NOTE: The following calls PrinterRail.set_position, 
            #       which calls set_position on each of the MCU_stepper objects 
            #       in each PrinterRail.
            # NOTE: This means that 4 calls will be made in total for a machine
            #       with X, Y, Y1, and Z steppers.
            # NOTE: This eventually calls "itersolve_set_position".
            rail.set_position(newpos)
            
            # NOTE: set limits if the axis is (being) homed.
            if i in homing_axes:
                # NOTE: This will put the axis to a "homed" state, which means that
                #       the unhomed part of the kinematic move check will pass from
                #       now on.
                logging.info(f"CartKinematics: setting limits={rail.get_range()} on stepper: {rail.get_name()}")
                self.limits[i] = rail.get_range()
            
    def note_z_not_homed(self):
        # Helper for Safe Z Home
        self.limits[2] = (1.0, -1.0)
    
    def home_axis(self, homing_state, axis, rail):
        # NOTE: "homing_state" is an instance of the "Homing" class.
        
        # Determine movement
        position_min, position_max = rail.get_range()
        hi = rail.get_homing_info()
        homepos = [None for i in range(self.toolhead_axis_count + 1)]
        homepos[axis] = hi.position_endstop
        forcepos = list(homepos)
        if hi.positive_dir:
            forcepos[axis] -= 1.5 * (hi.position_endstop - position_min)
        else:
            forcepos[axis] += 1.5 * (position_max - hi.position_endstop)
        
        # Perform homing
        logging.info(f"cartesian._home_axis: homing axis={axis} with forcepos={forcepos} and homepos={homepos}")
        homing_state.home_rails([rail], forcepos, homepos)
    
    def home(self, homing_state):
        # NOTE: "homing_state" is an instance of the "Homing" class.
        logging.info(f"cartesian.home: homing axis changed_axes={homing_state.changed_axes}")
        # Each axis is homed independently and in order
        for axis in homing_state.get_axes():
            if self.dc_module is not None and axis == self.dual_carriage_axis:
                self.dc_module.home(homing_state)
            else:
                self.home_axis(homing_state, axis, self.rails[axis])
    
    def _motor_off(self, print_time):
        self.limits = [(1.0, -1.0)] * 3
    
    def _check_endstops(self, move):
        logging.info(f"cartesian._check_endstops: triggered on {self.axis_names}/{self.axis} move.")
        end_pos = move.end_pos
        for i, axis in enumerate(self.axis):
            if (move.axes_d[axis]
                and (end_pos[axis] < self.limits[i][0]
                     or end_pos[axis] > self.limits[i][1])):
                if self.limits[i][0] > self.limits[i][1]:
                    # NOTE: self.limits will be "(1.0, -1.0)" when not homed, triggering this.
                    logging.info(f"cartesian._check_endstops: Must home axis {self.axis_names[i]} first.")
                    raise move.move_error(f"Must home axis {self.axis_names[i]} first")
                # NOTE: else raise a move error without a message.
                raise move.move_error()
    
    def check_move(self, move):
        limits = self.limits
        xpos, ypos = [move.end_pos[axis] for axis in self.axis[:2]]  # move.end_pos[:2]
        logging.info(f"cartesian.check_move: checking move ending on xpos={xpos} and ypos={ypos}.")
        if (xpos < limits[0][0] or xpos > limits[0][1]
            or ypos < limits[1][0] or ypos > limits[1][1]):
            self._check_endstops(move)
        if not move.axes_d[2]:
            # Normal XY move - use defaults
            return
        # Move with Z - update velocity and accel for slower Z axis
        self._check_endstops(move)
        z_ratio = move.move_d / abs(move.axes_d[2])
        move.limit_speed(
            self.max_z_velocity * z_ratio, self.max_z_accel * z_ratio)
    
    def get_status(self, eventtime):
        axes = [a for a, (l, h) in zip("xyz", self.limits) if l <= h]
        return {
            'homed_axes': "".join(axes),
            'axis_minimum': self.axes_min,
            'axis_maximum': self.axes_max,
        }

def load_kinematics(toolhead, config, trapq=None):
    return CartKinematics(toolhead, config, trapq)
