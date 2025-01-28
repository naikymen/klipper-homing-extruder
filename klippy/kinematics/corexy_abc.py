# Code for handling the kinematics of corexy robots
#
# Copyright (C) 2017-2021  Kevin O'Connor <kevin@koconnor.net>
# Copyright (C) 2023-2024  Nicolás A. Méndez
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import logging
from copy import deepcopy
from collections import namedtuple
import stepper

class CoreXYKinematicsABC:
    """CoreXY kinematics for the XYS or ABC axes in the main toolhead class.

    Example config for a corexy toolhead with 3 extra linear axes:
    
    [printer]
    kinematics: corexy_abc
    axis: XYZABC
    kinematics_abc: cartesian_abc
    max_velocity: 5000
    max_z_velocity: 250
    max_accel: 1000
    """ 
    def __init__(self, toolhead, config, trapq=None,
                 axes_ids=(3, 4, 5), axis_set_letters="ABC"):
        
        # Get the main printer object.
        self.printer = config.get_printer()

        # WARNING: ONLY THE FULL "XYZ" CONFIGURATION IS SUPPORTED IN COREXY FOR NOW.
        if len(axes_ids) != 3 or axis_set_letters.upper() != "XYZ":
            msg = "CoreXYKinematicsABC: Only the full 'XYZ' configuration is supported in corexy for now."
            raise self.printer.config_error(msg)
        
        # Save utilities for later.
        self.axes_to_xyz = toolhead.axes_to_xyz
        
        # Configured set of axes (indexes) and their letter IDs. Can have length less or equal to 3.
        self.axis_config = deepcopy(axes_ids)   # list of length <= 3: [0, 1, 3], [3, 4], [3, 4, 5], etc.
        self.axis_names = axis_set_letters      # char of length <= 3: "XYZ", "AB", "ABC", etc.
        self.axis_count = len(self.axis_names)  # integer count of configured axes (e.g. 2 for "XY").

        # Generate "local" indices (in the 0,1,2 range) for the kinematic's axes.
        self.axis_local = [i for i in range(self.axis_count)]  # Either [0, 1, 2], [0, 1] or [0].

        # Mapping dictionaries.
        self.axis_map = {k: v for k, v in zip(self.axis_names, self.axis_config)}       # {"x": 0}
        self.axis_map_rev = {v: k for k, v in zip(self.axis_names, self.axis_config)}   # {0: "x"}

        # Just to check
        if len(self.axis_config) != self.axis_count:
            msg = f"CartKinematicsABC: The amount of axis indexes in '{self.axis_config}'"
            msg += f" does not match the count of axis names '{self.axis_names}'."
            raise self.printer.config_error(msg)
        
        # NOTE: Infer the triplet from one of the axes: 1 means XYZ; 2 means ABC.
        triplet_number = axes_ids[0] // 3
        # NOTE: Full set of axes, forced to length 3. Starting at the first axis index (e.g. 0 for [0,1,2]),
        #       and ending at +3 (e.g. 3 for [0,1,2]).
        # NOTE: This attribute is used to select starting positions from a "move" object (see toolhead.py),
        #       which requires this list to have length 3 (because trapq_append needs the three components).
        # Example expected result: [0, 1, 2] for XYZ, [3, 4, 5] for ABC, [6, 7, 8] for UVW.
        self.axis = list(range(3*triplet_number, 3*triplet_number + 3))  # Length 3

        # Save which axes from the "triplet" will not have steppers configured.
        self.dummy_axes = list(set(self.axis).difference(self.axis_config))
        # Get the axis names of these "Dummy axes".
        self.dummy_axes_names = ["XYZABCUVW"[i] for i in self.dummy_axes]
        
        # Total axis count from the toolhead.
        self.toolhead_axis_count = toolhead.axis_count  # len(self.axis_names)
        
        # Report results of the multi-axis setup.
        msg = f"CoreXYKinematicsABC: starting setup with axes '{self.axis_names}'"
        msg += f", indexes '{self.axis_config}', and expanded indexes '{self.axis}'"
        logging.info(msg)
        
        if trapq is None:
            # Get the "trapq" object associated to the specified axes.
            self.trapq = toolhead.get_trapq(axes=self.axis_names)
        else:
            # Else use the provided trapq object.
            self.trapq = trapq
        
        # Setup axis rails
        # NOTE: A "PrinterRail" is setup by LookupMultiRail, per each 
        #       of the three axis, including their corresponding endstops.
        #       We do this by looking for "[stepper_?]" sections in the config.
        # NOTE: The "self.rails" list contains "PrinterRail" objects, which
        #       can have one or more stepper (PrinterStepper/MCU_stepper) objects.
        self.rails = [stepper.LookupMultiRail(config.getsection('stepper_' + n))
                      for n in self.axis_names.lower()]
        
        # NOTE: Iterate over the steppers in each of the two XY rails, and
        #       register each stepper in the endstop of the other rail.
        #       This I expect because endstops are likely placed in a 
        #       regular "cartesian" way, but in CoreXY both steppers must 
        #       move when homing to either the X or Y endstops.
        for s in self.rails[1].get_steppers():
            self.rails[0].get_endstops()[0][0].add_stepper(s)
        for s in self.rails[0].get_steppers():
            self.rails[1].get_endstops()[0][0].add_stepper(s)
        # NOTE: This probably associates each stepper to a particular solver,
        #       thar corresponds to the appropriate kinematics.
        self.rails[0].setup_itersolve('corexy_stepper_alloc', b'+')
        self.rails[1].setup_itersolve('corexy_stepper_alloc', b'-')
        self.rails[2].setup_itersolve('cartesian_stepper_alloc', b'z')
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
        
        # NOTE: Get "max_velocity" and "max_accel" from the toolhead's config.
        #       Used below as default values.
        max_velocity, max_accel = toolhead.get_max_velocity()
        self.max_z_velocity = config.getfloat('max_z_velocity', max_velocity, 
                                              above=0., maxval=max_velocity)
        self.max_z_accel = config.getfloat('max_z_accel', max_accel, 
                                           above=0., maxval=max_accel)
        
        # Setup boundary checks
        self.limits = [(1.0, -1.0)] * 3
        self.clear_homing_state((0, 1, 2))

        ranges = [r.get_range() for r in self.rails]
        # NOTE: Here I've swapped list expansion for dictionary expansion, and omitted "e",
        #       which will default to "None", and was previously forced to "0.0".
        #       See "cartesian_abc.py" for further detail.
        ranges_dict = dict()
        for a, r in zip(self.axis_names.lower(), ranges):
            ranges_dict[a] = r
        self.axes_min: namedtuple = toolhead.Coord(**{k: l for k, (l, h) in ranges_dict.items()})
        self.axes_max: namedtuple = toolhead.Coord(**{k: h for k, (l, h) in ranges_dict.items()})
    
    def get_steppers(self):
        return [s for rail in self.rails for s in rail.get_steppers()]
    
    def calc_position(self, stepper_positions):
        # Dummy default position.
        pos = [0.0 for i in range(3)]
        # Replace defaults.
        for i, axis in enumerate(self.axis_config):
            # e.g.: Get rail "stepper_a", with axis name "A", and axis index "3".
            # We know that the "axis_config" and "rails" lists match in order
            # because this is how they were defined during init in this class.
            rail = self.rails[i]
            stepper_position = stepper_positions[rail.get_name()]
            # Axis may be from XYZ, ABC, or higher-indexed triplets.
            # Get the remainder of the current axis to convert it to an
            # index between 0 and 2, which can be used to overwrite the
            # "dummy" position (i.e. 0.0 above).
            pos[axis % 3] = stepper_position
        # Convert CoreXY stepper positions to cartesian XY coordinates.
        xyz_pos = [0.5 * (pos[0] + pos[1]), 0.5 * (pos[0] - pos[1]), pos[2]]
        return xyz_pos
    
    def set_position(self, newpos, homing_axes):
        for i, rail in enumerate(self.rails):
            rail.set_position(newpos)
            if i in homing_axes:
                self.limits[i] = rail.get_range()
    
    def clear_homing_state(self, axes):
        # TODO: Check if I need to use a mapping key here (e.g. self.axis_map["Z"]).
        for i, _ in enumerate(self.limits):
            if i in axes:
                self.limits[i] = (1.0, -1.0)

    def home(self, homing_state):
        # Each axis is homed independently and in order
        for axis in homing_state.get_axes():
            rail_index = self.axes_to_xyz(axis)
            self.home_axis(homing_state, axis, self.rails[rail_index])
    
    def home_axis(self, homing_state, axis, rail):
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
        homing_state.home_rails([rail], forcepos, homepos)
    
    def _check_endstops(self, move):
        end_pos = move.end_pos
        for i, axis in enumerate(self.axis_config):
            if (move.axes_d[axis]
                and (end_pos[axis] < self.limits[i][0]
                     or end_pos[axis] > self.limits[i][1])):
                if self.limits[i][0] > self.limits[i][1]:
                    # NOTE: self.limits will be "(1.0, -1.0)" when not homed, triggering this.
                    msg = f"corexy_abc._check_endstops: Must home axis {self.axis_names[i]} first,"
                    msg += f"limits={self.limits[i]} end_pos[axis]={end_pos[axis]} "
                    msg += f"move.axes_d[axis]={move.axes_d[axis]}"
                    logging.info(msg)
                    raise move.move_error(f"Must home axis {self.axis_names[i]} first")
                raise move.move_error()
    
    def check_move(self, move):
        limit_checks = []
        for i, axis in enumerate(self.axis_config):
            # TODO: Check if its better to iterate over "self.axis" instead,
            #       see rationale in favor of "axis_config" above, at "_check_endstops".
            pos = move.end_pos[axis]
            limit_checks.append(pos < self.limits[i][0] or pos > self.limits[i][1])
        if any(limit_checks):
            self._check_endstops(move)
        
        # TODO: Update this part of the code to handle 
        #       the case when Z axis is not configured.
        if not move.axes_d[2]:
            # Normal XY move - use defaults
            return
        # Move with Z - update velocity and accel for slower Z axis
        self._check_endstops(move)
        z_ratio = move.move_d / abs(move.axes_d[2])
        move.limit_speed(
            self.max_z_velocity * z_ratio, self.max_z_accel * z_ratio)
    
    def get_status(self, eventtime):
        # NOTE: If you alter this you should probably
        #       do so also in the other "abc" kinematics.
        axes = [a for a, (l, h) in zip(self.axis_names.lower(), self.limits) if l <= h]
        return {
            'homed_axes': "".join(axes),
            'axis_minimum': self.axes_min,
            'axis_maximum': self.axes_max,
        }

def load_kinematics(toolhead, config, trapq=None, axes_ids=(0, 1, 2), axis_set_letters="XYZ"):
    return CoreXYKinematicsABC(toolhead, config, trapq, axes_ids, axis_set_letters)
