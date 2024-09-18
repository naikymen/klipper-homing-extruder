# adds support fro ARC commands via G2/G3
#
# Copyright (C) 2019  Aleksej Vasiljkovic <achmed21@gmail.com>
#
# function planArc() originates from https://github.com/MarlinFirmware/Marlin
# Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math, logging, copy

# Setup a logger: https://stackoverflow.com/a/11233293
def setup_logger(name, log_file, level=logging.INFO):
    """To setup as many loggers as you want"""
    handler = logging.FileHandler(log_file)        

    # formatter = logging.Formatter('%(asctime)s %(levelname)s %(message)s')
    # handler.setFormatter(formatter)

    logger = logging.getLogger(name)
    logger.setLevel(level)
    logger.addHandler(handler)

    return logger
gcode_log = setup_logger('gcode_log', '/tmp/gcode.log')


# Coordinates created by this are converted into G1 commands.
#
# supports XY, XZ & YZ planes with remaining axis as helical

class ArcSupport:
 
    def __init__(self, config):
        """Support for gcode arc (G2/G3) commands.
        [gcode_arcs]
        #resolution: 1.0
        #   An arc will be split into segments. Each segment's length will
        #   equal the resolution in mm set above. Lower values will produce a
        #   finer arc, but also more work for your machine. Arcs smaller than
        #   the configured value will become straight lines. The default is
        #   1 mm.

        To start adding support for multi-axis, this reads the 'axis' parameter
        of the '[printer]' section in the config file:
        [printer]
        # ...
        axis: XYZ  # Optional: XYZ or XYZABC
        # ...

        Example GCODE to test. The following commands should complete in the same amount of time:
        - G2 X10 Y10 I10 J0 F10000
        - G2 X10 Y10 A10 I10 J0 F10000
        """
        
        self.printer = config.get_printer()
        self.mm_per_arc_segment = config.getfloat('resolution', 1., above=0.0)

        self.gcode_move = self.printer.load_object(config, 'gcode_move')
        self.gcode = self.printer.lookup_object('gcode')
        
        # Get amount of axes
        # NOTE: Amount of non-extruder axes: XYZ=3, XYZABC=6.
        self.axis_names = config.getsection("printer").get('axis', 'XYZ')  # "XYZ" / "XYZABC"
        self.axis_count = len(self.axis_names)
        # Axis sets and names for them are partially hardcoded all around.
        self.axis_triplets = ["XYZ", "ABC", "UVW"]
        self.ax_letters = "".join(self.axis_triplets)
        # Find the minimum amount of axes needed for the requested axis triplets.
        # For example, 1 triplet would be required for "XYZ" or "ABC", but 2
        # triplets are needed for any mixing of those (e.g. "XYZAB").
        self.min_axes = 3 * sum([ 1 for axset in self.axis_triplets if set(self.axis_names).intersection(axset) ])
        # Length for the position vector, matching the required axis,
        # plus 1 for the extruder axis (even if it is a dummy one).
        self.pos_length = self.min_axes + 1
        # self.pos_length = self.axis_count + 1
        # NOTE: The value of this attriute must match the one at "toolhead.py".

        # Dictionary to map axes to their indexes in the position vector.
        self.axis_map = {a: i for i, a in enumerate(list(self.ax_letters)[:self.min_axes] + ["E"])}

        # Enum
        self.ARC_PLANE_X_Y = 0
        self.ARC_PLANE_X_Z = 1
        self.ARC_PLANE_Y_Z = 2

        # Enum
        self.X_AXIS = self.axis_map.get("X", 0)
        self.Y_AXIS = self.axis_map.get("Y", 1)
        self.Z_AXIS = self.axis_map.get("Z", 2)
        # self.A_AXIS = self.axis_map.get("A", 4)  # NOTE: Not used below.
        self.E_AXIS = self.axis_map.get("E", None)  # self.min_axes
        
        # Register g-code commands
        self.gcode.register_command("G2", self.cmd_G2)
        self.gcode.register_command("G3", self.cmd_G3)

        self.gcode.register_command("G17", self.cmd_G17)
        self.gcode.register_command("G18", self.cmd_G18)
        self.gcode.register_command("G19", self.cmd_G19)

        # This is a named tuple with elements: ('x', 'y', 'z', 'e', 'a', 'b', 'c')
        # Values default to None.
        self.Coord = self.gcode.Coord

        # backwards compatibility, prior implementation only supported XY
        self.plane = self.ARC_PLANE_X_Y

    cmd_G2_help = "Clockwise arc move to a specified position."
    def cmd_G2(self, gcmd):
        """Arc Move Clockwise: G2 [X<pos>] [Y<pos>] [Z<pos>] [E<pos>] [F<speed>] I<value> J<value>|I<value> K<value>|J<value> K<value>"""
        self._cmd_inner(gcmd, True)

    cmd_G3_help = "Counter-clockwise arc move to a specified position."
    def cmd_G3(self, gcmd):
        """Arc Move Counter-clockwise: G3 [X<pos>] [Y<pos>] [Z<pos>] [E<pos>] [F<speed>] I<value> J<value>|I<value> K<value>|J<value> K<value>"""
        self._cmd_inner(gcmd, False)

    cmd_G17_help = "Select XY plane for circular interpolation"
    def cmd_G17(self, gcmd):
        """Arc Plane Select: G17 (XY plane)"""
        self.plane = self.ARC_PLANE_X_Y

    cmd_G18_help = "Select XZ plane for circular interpolation"
    def cmd_G18(self, gcmd):
        """Arc Plane Select: G18 (XZ plane)"""
        self.plane = self.ARC_PLANE_X_Z

    cmd_G19_help = "Select YZ plane for circular interpolation"
    def cmd_G19(self, gcmd):
        """Arc Plane Select: G19 (YZ plane)"""
        self.plane = self.ARC_PLANE_Y_Z

    def _cmd_inner(self, gcmd, clockwise):
        # The arc's path is planned in absolute coordinates.
        gcodestatus = self.gcode_move.get_status()
        if not gcodestatus['absolute_coordinates']:
            raise gcmd.error("G2/G3 does not support relative move mode")
        currentPos = gcodestatus['gcode_position']
        absolut_extrude = gcodestatus['absolute_extrude']

        # Parse parameters
        asTarget = [gcmd.get_float("X", currentPos[0]),
                    gcmd.get_float("Y", currentPos[1]),
                    gcmd.get_float("Z", currentPos[2])]

        if gcmd.get_float("R", None) is not None:
            raise gcmd.error("G2/G3 does not support R moves")

        # determine the plane coordinates and the helical axis
        I = gcmd.get_float('I', 0.)
        J = gcmd.get_float('J', 0.)
        asPlanar = (I, J)
        axes = (self.X_AXIS, self.Y_AXIS, self.Z_AXIS)
        if self.plane == self.ARC_PLANE_X_Z:
            K = gcmd.get_float('K', 0.)
            asPlanar = (I, K)
            axes = (self.X_AXIS, self.Z_AXIS, self.Y_AXIS)
        elif self.plane == self.ARC_PLANE_Y_Z:
            K = gcmd.get_float('K', 0.)
            asPlanar = (J, K)
            axes = (self.Y_AXIS, self.Z_AXIS, self.X_AXIS)

        if not (asPlanar[0] or asPlanar[1]):
            raise gcmd.error("G2/G3 requires IJ, IK or JK parameters")

        # Build linear coordinates to move
        self.planArc(currentPos, asTarget, asPlanar, clockwise,
                     gcmd, absolut_extrude, *axes)

    # function planArc() originates from marlin plan_arc()
    # https://github.com/MarlinFirmware/Marlin
    #
    # The arc is approximated by generating many small linear segments.
    # The length of each segment is configured in MM_PER_ARC_SEGMENT
    # Arcs smaller than this value will be a Line only
    #
    # alpha and beta axes are the current plane, helical axis is linear travel
    def planArc(self, currentPos, targetPos, offset, clockwise,
                gcmd, absolut_extrude,
                alpha_axis, beta_axis, helical_axis):
        # todo: sometimes produces full circles

        # Radius vector from center to current location
        r_P = -offset[0]
        r_Q = -offset[1]

        # Determine angular travel
        center_P = currentPos[alpha_axis] - r_P
        center_Q = currentPos[beta_axis] - r_Q
        rt_Alpha = targetPos[alpha_axis] - center_P
        rt_Beta = targetPos[beta_axis] - center_Q
        angular_travel = math.atan2(r_P * rt_Beta - r_Q * rt_Alpha,
                                    r_P * rt_Alpha + r_Q * rt_Beta)
        if angular_travel < 0.:
            angular_travel += 2. * math.pi
        if clockwise:
            angular_travel -= 2. * math.pi

        if (angular_travel == 0.
            and currentPos[alpha_axis] == targetPos[alpha_axis]
            and currentPos[beta_axis] == targetPos[beta_axis]):
            # Make a circle if the angular rotation is 0 and the
            # target is the current position
            angular_travel = 2. * math.pi

        # Determine the number of segments
        linear_travel = targetPos[helical_axis] - currentPos[helical_axis]
        radius = math.hypot(r_P, r_Q)
        flat_mm = radius * angular_travel
        if linear_travel:
            mm_of_travel = math.hypot(flat_mm, linear_travel)
        else:
            mm_of_travel = math.fabs(flat_mm)
        segments = max(1., math.floor(mm_of_travel / self.mm_per_arc_segment))

        # Generate coordinates
        theta_per_segment = angular_travel / segments
        linear_per_segment = linear_travel / segments

        asE = gcmd.get_float("E", None)
        asF = gcmd.get_float("F", None)

        # NOTE: Add support for moves in the ABC axes, which are not 
        #       part of the arc move, and should be split like extruder
        #       moves or "helical axis" moves are.
        g1_abc_params = {}
        g1_abc_increments = {}
        abc_sq_displacement = 0.0  # Sum of the squared ABC displacements. 
        extra_axes_idx = {ax: idx for ax, idx in self.axis_map.items() if idx not in [0,1,2,self.E_AXIS]}
        for axis_letter, idx in extra_axes_idx.items():
            axis_coord = gcmd.get_float(axis_letter, None)
            if axis_coord is not None:
                # Calculate the distance this axis will move during each arc segment.
                axis_increment = axis_coord / segments
                # Calculate sum of square displacements for the extra axes.
                abc_sq_displacement += axis_increment ** 2
                # Make target coordinates for the extra axes of the first move.
                g1_abc_params[axis_letter] = currentPos[idx] + axis_increment
                g1_abc_increments[axis_letter] = axis_increment

        # NOTE: Prepare a command that restores the original feedrate.
        feedrate = self.gcode_move.speed * 1.0
        if asF is not None:
            feedrate = asF
        g1_f_gcmd = self.gcode.create_gcode_command(command="G1", commandline="G1", params={"F": feedrate})

        e_per_move = e_base = 0.
        if asE is not None:
            if absolut_extrude:
                e_base = currentPos[3]
            e_per_move = (asE - e_base) / segments

        # Initialize previous position vector.
        c_prev = [None for i in range(self.pos_length)]
        c_prev[alpha_axis] = center_P
        c_prev[beta_axis] = center_Q
        c_prev[helical_axis] = currentPos[helical_axis]

        for i in range(1, int(segments) + 1):
            dist_Helical = i * linear_per_segment
            c_theta = i * theta_per_segment
            cos_Ti = math.cos(c_theta)
            sin_Ti = math.sin(c_theta)
            r_P = -offset[0] * cos_Ti + offset[1] * sin_Ti
            r_Q = -offset[0] * sin_Ti - offset[1] * cos_Ti

            # Dummy position vector.
            c = [None for i in range(self.pos_length)]
            c[alpha_axis] = center_P + r_P
            c[beta_axis] = center_Q + r_Q
            c[helical_axis] = currentPos[helical_axis] + dist_Helical

            # Force target position if this is the last segment.
            if i == segments:
                c = targetPos

            # Convert coords into G1 commands
            g1_params = {'X': c[0], 'Y': c[1], 'Z': c[2]}
            if e_per_move:
                g1_params['E'] = e_base + e_per_move
                if absolut_extrude:
                    e_base += e_per_move

            # NOTE: Add support for linear moves in the ABC axes.
            for ax, pos in g1_abc_params.items():
                g1_params[ax] = pos
                g1_abc_params[ax] += g1_abc_increments[ax]

            # NOTE: Calculate feedrate adjustment factor for the current move.
            #       This is used to "increase" the feedrate internally,
            #       because: (1) the feedrate for arc moves is expected
            #       to affect to non-ABC moves only (2) Klipper will split
            #       the "available feedrate" among all axes; except the E axis.
            xyz_sq_displacement = sum([(c_next-c_prev)**2 for c_next, c_prev in zip(c[0:3], c_prev[0:3])])
            feedrate_factor = math.sqrt(xyz_sq_displacement + abc_sq_displacement) / math.sqrt(xyz_sq_displacement)

            # Set the adjusted feedrate.
            g1_params['F'] = feedrate * feedrate_factor

            g1_gcmd = self.gcode.create_gcode_command("G1", "G1", g1_params)

            # NOTE: write actual G1 commands to the log.
            # gcode_log.info( f'G1 { " ".join([f"{k}{v}" for k, v in g1_params.items()]) }; >>> Arc segment with target: {asTarget}' )
            gcode_log.info( f'G1 { " ".join([f"{k}{v}" for k, v in g1_params.items()]) }' )

            # Send the command to the move queue.
            self.gcode_move.cmd_G1(g1_gcmd)

            # Save new coordinate for the next loop iteration.
            c_prev = c.copy()

        # NOTE: Restore original feedrate.
        self.gcode_move.cmd_G1(g1_f_gcmd)

def load_config(config):
    return ArcSupport(config)
