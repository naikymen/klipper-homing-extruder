# Perform Z Homing at specific XY coordinates.
#
# Copyright (C) 2019 Florian Heilmann <Florian.Heilmann@gmx.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

class SafeZHoming:
    def __init__(self, config):
        self.printer = config.get_printer()
        x_pos, y_pos = config.getfloatlist("home_xy_position", count=2)
        self.home_x_pos, self.home_y_pos = x_pos, y_pos
        self.z_hop = config.getfloat("z_hop", default=0.0)
        self.z_hop_speed = config.getfloat('z_hop_speed', 15., above=0.)
        zconfig = config.getsection('stepper_z')
        self.max_z = zconfig.getfloat('position_max', note_valid=False)
        self.speed = config.getfloat('speed', 50.0, above=0.)
        self.move_to_previous = config.getboolean('move_to_previous', False)
        self.printer.load_object(config, 'homing')
        self.gcode = self.printer.lookup_object('gcode')
        self.prev_G28 = self.gcode.register_command("G28", None)
        self.gcode.register_command("G28", self.cmd_G28, desc=self.cmd_G28_help)

        if config.has_section("homing_override"):
            raise config.error("homing_override and safe_z_homing cannot"
                               +" be used simultaneously")

    cmd_G28_help = "Performing Homing procedure with Safe Z Home"
    def cmd_G28(self, gcmd):
        toolhead = self.printer.lookup_object('toolhead')
        z_axis_idx: int = toolhead.axis_map["Z"]

        # Perform Z Hop if necessary
        if self.z_hop != 0.0:
            # Check if Z axis is homed and its last known position
            curtime = self.printer.get_reactor().monotonic()
            kin_status = toolhead.get_kinematics().get_status(curtime)
            pos = toolhead.get_position()

            if 'z' not in kin_status['homed_axes']:
                # Always perform the z_hop if the Z axis is not homed
                pos[z_axis_idx] = 0
                toolhead.set_position(pos, homing_axes=[z_axis_idx])
                toolhead.manual_move([None, None, self.z_hop],
                                     self.z_hop_speed)
                if hasattr(toolhead.get_kinematics(), "note_z_not_homed"):
                    toolhead.get_kinematics().note_z_not_homed()
            elif pos[z_axis_idx] < self.z_hop:
                # If the Z axis is homed, and below z_hop, lift it to z_hop
                toolhead.manual_move([None, None, self.z_hop],
                                     self.z_hop_speed)

        # Determine which axes we need to home
        need_axes = [gcmd.get(axis, None) is not None for axis in toolhead.axis_names]
        # If the list is empty, or every item is None, home everything.
        home_all = not any(need_axes)

        # Home XY axes if necessary
        new_params = {}
        for a in need_axes:
            if a is None or a == 'Z':
                continue
            else:
                new_params[a] = '0'
        if new_params:
            g28_gcmd = self.gcode.create_gcode_command("G28", "G28", new_params)
            self.prev_G28(g28_gcmd)

        # Home Z axis if necessary
        if ('Z' in need_axes) or home_all:
            # Throw an error if X or Y are not homed
            curtime = self.printer.get_reactor().monotonic()
            # TODO: Adapt this to correctly identify the required kinematics.
            kin_status = toolhead.get_kinematics(axes="XYZ").get_status(curtime)
            if ('x' not in kin_status['homed_axes'] or
                'y' not in kin_status['homed_axes']):
                raise gcmd.error("Must home X and Y axes first")
            # Move to safe XY homing position
            prevpos = toolhead.get_position()
            toolhead.manual_move([self.home_x_pos, self.home_y_pos], self.speed)
            # Home Z
            g28_gcmd = self.gcode.create_gcode_command("G28", "G28", {'Z': '0'})
            self.prev_G28(g28_gcmd)
            # Perform Z Hop again for pressure-based probes
            if self.z_hop:
                pos = toolhead.get_position()
                if pos[z_axis_idx] < self.z_hop:
                    toolhead.manual_move([None, None, self.z_hop],
                                         self.z_hop_speed)
            # Move XY back to previous positions
            if self.move_to_previous:
                xy_idxs = [toolhead.axis_map[a] for a in "XY"]
                toolhead.manual_move(prevpos[xy_idxs], self.speed)

def load_config(config):
    return SafeZHoming(config)
