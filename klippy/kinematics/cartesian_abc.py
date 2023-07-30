# Code for handling the kinematics of cartesian robots
#
# Copyright (C) 2016-2021  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
import stepper
from kinematics.cartesian import CartKinematics
from copy import deepcopy

class CartKinematicsABC(CartKinematics):
    """Kinematics for the ABC axes in the main toolhead class.

    Example config:
    
    [printer]
    kinematics: cartesian
    axis: XYZ  # Optional: XYZ or XYZABC
    kinematics_abc: cartesian_abc # Optional
    max_velocity: 5000
    max_z_velocity: 250
    max_accel: 1000
    
    TODO:
      - The "checks" still have the XYZ logic.
      - Homing is not implemented for ABC.
    """    
    def __init__(self, toolhead, config, trapq=None,
                 axes_ids=(3, 4), axis_set_letters="AB"):
        """Cartesian kinematics.
        
        Configures up to 3 cartesian axes, or less.

        Args:
            toolhead (_type_): Toolhead-like object.
            config (_type_): Toolhead-like config object.
            trapq (_type_, optional): Trapq object. Defaults to None.
            axes_ids (tuple, optional): Configured set of integer axes IDs. Can have length less than 3. Defaults to (3, 4).
            axis_set_letters (str, optional): Configured set of letter axes IDs. Can have length less than 3. Defaults to "AB".
        """
        self.printer = config.get_printer()
        
        
        # Configured set of axes (indexes) and their letter IDs. Can have length less or equal to 3.
        self.axis_config = deepcopy(axes_ids)  # list of length <= 3: [0, 1, 3], [3, 4], [3, 4, 5], etc.
        self.axis_names = axis_set_letters  # char of length <= 3: "XYZ", "AB", "ABC", etc.
        self.axis_count = len(self.axis_names)

        # Just to check
        if len(self.axis_config) != self.axis_count:
            msg = f"CartKinematicsABC: The amount of axis indexes in '{self.axis_config}'"
            msg += f" does not match the count of axis names '{self.axis_names}'."
            raise Exception(msg)
        
        # Full set of axes, forced to length 3. Starting at the first axis index (e.g. 0 for [0,1,2]),
        # and ending at +3 (e.g. 3 for [0,1,2]).
        # Example expected result: [0, 1, 2] for XYZ, [3, 4, 5] for ABC, [6, 7, 8] for UVW.
        self.axis = list(range(self.axis_config[0], self.axis_config[0] + 3))  # Length 3
        
        # Total axis count from the toolhead.
        self.toolhead_axis_count = toolhead.axis_count  # len(self.axis_names)
        
        # Report
        msg = f"\n\nCartKinematicsABC: starting setup with axes '{self.axis_names}'"
        msg += f", indexes '{self.axis_config}', and expanded indexes '{self.axis}'\n\n"
        logging.info(msg)
        
        if trapq is None:
            # Get the "trapq" object associated to the specified axes.
            self.trapq = toolhead.get_trapq(axes=self.axis_names)
        else:
            # Else use the provided trapq object.
            self.trapq = trapq
        
        # Setup axis rails. DISABLED!
        # self.dual_carriage_axis = None
        # self.dual_carriage_rails = []
        
        # NOTE: A "PrinterRail" is setup by LookupMultiRail, per each 
        #       of the three axis, including their corresponding endstops.
        #       We do this by looking for "[stepper_?]" sections in the config.
        # NOTE: The "self.rails" list contains "PrinterRail" objects, which
        #       can have one or more stepper (PrinterStepper/MCU_stepper) objects.
        self.rails = [stepper.LookupMultiRail(config.getsection('stepper_' + n))
                      for n in self.axis_names.lower()]
        
        # NOTE: "xyz_axis_names" must always be "xyz" and not "abc", 
        #       see "cartesian_stepper_alloc" in C code.
        # TODO: Check if it also needs to be length 3 every time.
        #       The call to "setup_itersolve" for a manual stepper
        #       is only done once, to setup an "x" axis.
        # NOTE: Can be "xyz", "xy", or just "x". This does not need to correspond
        #       to the actual axis names, the intuition is to mimic the manual stepper
        #       setup, starting with just "x", and then allow more axes to be setup.
        xyz_axis_names = "xyz"[:len(self.axis_names)]
        for rail, axis in zip(self.rails, xyz_axis_names):
            rail.setup_itersolve('cartesian_stepper_alloc', axis.encode())
        
        # NOTE: Iterates over "self.rails" to get all the stepper objects.
        for s in self.get_steppers():
            # NOTE: Each "s" stepper is an "MCU_stepper" object.
            s.set_trapq(self.trapq)
            # NOTE: This object is used by "toolhead._update_move_time".
            toolhead.register_step_generator(s.generate_steps)
            # TODO: Check if this "generator" should be appended to 
            #       the "self.step_generators" list in the toolhead,
            #       or to the list in the new TrapQ...
            #       Using the toolhead for now.
        
        # Register a handler for turning off the steppers.
        self.printer.register_event_handler("stepper_enable:motor_off",
                                            self._motor_off)
        
        # NOTE: Returns max_velocity and max_accel from the toolhead's config.
        #       Used below as default values.
        max_velocity, max_accel = toolhead.get_max_velocity()
        self.max_z_velocity = config.getfloat('max_z_velocity', max_velocity,
                                              above=0., maxval=max_velocity)
        self.max_z_accel = config.getfloat('max_z_accel', max_accel,
                                           above=0., maxval=max_accel)
        
        # Setup limits.
        self.reset_limits()
        
        # Setup boundary checks.
        ranges = [r.get_range() for r in self.rails]
        # TODO: Check that this works with ABC axes, it will result in 
        #       "Coord(x=1.0, y=0.0, z=0.0, e=0.0, a=None, b=None, c=None)"
        #       "Coord(x=-1.0, y=-1.0, z=-1.0, e=0.0, a=None, b=None, c=None)"
        self.axes_min = toolhead.Coord(*[r[0] for r in ranges], e=0.)
        self.axes_max = toolhead.Coord(*[r[1] for r in ranges], e=0.)
        
        # Check for dual carriage support
        # if config.has_section('dual_carriage'):
        #     dc_config = config.getsection('dual_carriage')
        #     dc_axis = dc_config.getchoice('axis', {'x': 'x', 'y': 'y'})
        #     self.dual_carriage_axis = {'x': 0, 'y': 1}[dc_axis]
        #     dc_rail = stepper.LookupMultiRail(dc_config)
        #     dc_rail.setup_itersolve('cartesian_stepper_alloc', dc_axis.encode())
        #     for s in dc_rail.get_steppers():
        #         toolhead.register_step_generator(s.generate_steps)
        #     self.dual_carriage_rails = [
        #         self.rails[self.dual_carriage_axis], dc_rail]
        #     self.printer.lookup_object('gcode').register_command(
        #         'SET_DUAL_CARRIAGE', self.cmd_SET_DUAL_CARRIAGE,
        #         desc=self.cmd_SET_DUAL_CARRIAGE_help)
    
    def reset_limits(self):
        # self.limits = [(1.0, -1.0)] * len(self.axis_config)
        # TODO: Should this have length < 3 if less axes are configured, or not?
        #       CartKinematics methods like "get_status" will expect length 3 limits.
        #       See "get_status" for more details.
        # NOTE: Using length 3
        self.limits = [(1.0, -1.0)] * 3
        # NOTE: I've got all of the (internal) calls covered.
        #       There may be other uses of the "limits" attribute elsewhere.
    
    def get_steppers(self):
        # NOTE: The "self.rails" list contains "PrinterRail" objects, which
        #       can have one or more stepper (PrinterStepper/MCU_stepper) objects.
        rails = self.rails
        
        # if self.dual_carriage_axis is not None:
        #     dca = self.dual_carriage_axis
        #     rails = rails[:dca] + self.dual_carriage_rails + rails[dca+1:]
        
        # NOTE: run "get_steppers" on each "PrinterRail" object from 
        #       the "self.rails" list. That method returns the list of
        #       all "PrinterStepper"/"MCU_stepper" objects in the kinematic.
        return [s for rail in rails for s in rail.get_steppers()]
    
    def calc_position(self, stepper_positions):
        return [stepper_positions[rail.get_name()] for rail in self.rails]
    
    def set_position(self, newpos, homing_axes):
        logging.info("\n\n" +
                     f"CartKinematicsABC.set_position: setting kinematic position of {len(self.rails)} rails " +
                     f"with newpos={newpos} and homing_axes={homing_axes}\n\n")
        for i, rail in enumerate(self.rails):
            logging.info(f"\n\nCartKinematicsABC: setting newpos={newpos} on stepper: {rail.get_name()}\n\n")
            rail.set_position(newpos)
            if i in homing_axes:
                logging.info(f"\n\nCartKinematicsABC: setting limits={rail.get_range()} on stepper: {rail.get_name()}\n\n")
                # NOTE: Here each limit becomes associated to a certain "rail" (i.e. an axis).
                #       If the rails were set up as "XYZ" in that order (as per "self.axis_names"),
                #       the limits will now correspond to them in that same order.
                # NOTE: This is relevant fot "get_status".
                self.limits[i] = rail.get_range()
    
    def note_z_not_homed(self):
        # Helper for Safe Z Home
        # self.limits[2] = (1.0, -1.0)
        # TODO: reconsider ignoring the call, it can be produced by "safe_z_home".
        logging.info(f"\n\nCartKinematicsABC WARNING: call to note_z_not_homed ignored.\n\n")
        pass
    
    def _home_axis(self, homing_state, axis, rail):
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
        logging.info(f"\n\ncartesian_abc._home_axis: homing axis={axis} with forcepos={forcepos} and homepos={homepos}\n\n")
        homing_state.home_rails([rail], forcepos, homepos)
    
    def home(self, homing_state):
        # Each axis is homed independently and in order
        toolhead = self.printer.lookup_object('toolhead')
        for axis in homing_state.get_axes():
            # NOTE: support for dual carriage removed.
            self._home_axis(homing_state, axis, self.rails[toolhead.axes_to_xyz(axis)])
    
    def _motor_off(self, print_time):
        self.reset_limits()
    
    def _check_endstops(self, move):
        logging.info("\n\n" + f"cartesian_abc._check_endstops: triggered on {self.axis_names}/{self.axis} move.\n\n")
        end_pos = move.end_pos
        for i, axis in enumerate(self.axis_config):
            # TODO: Check if its better to iterate over "self.axis" instead,
            #       which is forced to lenght 3. For now "self.axis_config"
            #       seems more reasonable, as it will be the toolhead passing
            #       the move, and it was the toolhead that specified the axis
            #       indices for this kinematic during setup in the first place.
            #       Furthermore, limits are ordered by "self.axis_names", which
            #       correlates 1:1 with "self.axis_config".
            if (move.axes_d[axis]
                and (end_pos[axis] < self.limits[i][0]
                     or end_pos[axis] > self.limits[i][1])):
                if self.limits[i][0] > self.limits[i][1]:
                    # NOTE: self.limits will be "(1.0, -1.0)" when not homed, triggering this.
                    msg = "".join(["\n\n" + f"cartesian_abc._check_endstops: Must home axis {self.axis_names[i]} first,",
                                   f"limits={self.limits[i]} end_pos[axis]={end_pos[axis]} ",
                                   f"move.axes_d[axis]={move.axes_d[axis]}" + "\n\n"])
                    logging.info(msg)
                    raise move.move_error(f"Must home axis {self.axis_names[i]} first")
                raise move.move_error()
    
    # TODO: Use the original toolhead's z-axis limit here.
    # TODO: Think how to "sync" speeds with the original toolhead,
    #       so far the ABC axis should just mirror the XY.
    def check_move(self, move):
        """Checks a move for validity.
        
        Also limits the move's max speed to the limit of the Z axis if used.

        Args:
            move (tolhead.Move): Instance of the Move class.
        """
        limit_checks = []
        for i, axis in enumerate(self.axis_config):
            # TODO: Check if its better to iterate over "self.axis" instead,
            #       see rationale in favor of "axis_config" above, at "_check_endstops".
            pos = move.end_pos[axis]
            limit_checks.append(pos < self.limits[i][0] or pos > self.limits[i][1])
        if any(limit_checks):
            self._check_endstops(move)
        
        # limits = self.limits
        # apos, bpos = [move.end_pos[axis] for axis in self.axis[:2]]  # move.end_pos[3:6]
        # logging.info("\n\n" + f"cartesian_abc.check_move: checking move ending on apos={apos} and bpos={bpos}.\n\n")
        # if (apos < limits[0][0] or apos > limits[0][1]
        #     or bpos < limits[1][0] or bpos > limits[1][1]):
        #     self._check_endstops(move)
        
        self._check_endstops(move)
        
        # TODO: Reconsider adding Z-axis speed limiting.
        # # NOTE: check if the move involves the Z axis, to limit the speed.
        # if not move.axes_d[self.axis[2]]:
        #     # Normal XY move, no Z axis movements - use default speed.
        #     return
        # else:
        #     pass
        #     # NOTE: removed the "Z" logic here, as it is implemented in 
        #     #       the XYZ cartesian kinematic check already.
        #     # Move with Z - update velocity and accel for slower Z axis
        #     # z_ratio = move.move_d / abs(move.axes_d[2])
        #     # move.limit_speed(
        #     #     self.max_z_velocity * z_ratio, self.max_z_accel * z_ratio)
        return
    
    def get_status(self, eventtime):
        # NOTE: "zip" will iterate until one of the arguments runs out.
        #       This means that having "XY" axis names is not problematic
        #       when self.limits is length 3, and viceversa.
        axes = [a for a, (l, h) in zip(self.axis_names.lower(), self.limits) if l <= h]
        return {
            'homed_axes': "".join(axes),
            'axis_minimum': self.axes_min,
            'axis_maximum': self.axes_max,
        }
    
    # Dual carriage support
    # def _activate_carriage(self, carriage):
    #     toolhead = self.printer.lookup_object('toolhead')
    #     toolhead.flush_step_generation()
    #     dc_rail = self.dual_carriage_rails[carriage]
    #     dc_axis = self.dual_carriage_axis
    #     self.rails[dc_axis].set_trapq(None)
    #     dc_rail.set_trapq(toolhead.get_trapq())
    #     self.rails[dc_axis] = dc_rail
    #     pos = toolhead.get_position()
    #     pos[dc_axis] = dc_rail.get_commanded_position()
    #     toolhead.set_position(pos)
    #     if self.limits[dc_axis][0] <= self.limits[dc_axis][1]:
    #         self.limits[dc_axis] = dc_rail.get_range()
    # cmd_SET_DUAL_CARRIAGE_help = "Set which carriage is active"
    # def cmd_SET_DUAL_CARRIAGE(self, gcmd):
    #     carriage = gcmd.get_int('CARRIAGE', minval=0, maxval=1)
    #     self._activate_carriage(carriage)

def load_kinematics(toolhead, config, trapq=None, axes_ids=(0, 1, 2), axis_set_letters="XYZ"):
    return CartKinematicsABC(toolhead, config, trapq, axes_ids, axis_set_letters)
