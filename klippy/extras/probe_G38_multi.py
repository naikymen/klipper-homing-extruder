# Z-Probe support
#
# Copyright (C) 2017-2021  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

# pylint: disable=missing-class-docstring,missing-function-docstring,invalid-name,line-too-long,consider-using-f-string
# pylint: disable=logging-fstring-interpolation,logging-not-lazy,fixme

import logging
from . import probe, probe_G38

class ProbeG38multi(probe_G38.ProbeG38):
    def __init__(self, config):
        # NOTE: because the "config" is passed to PrinterProbe and ProbeEndstopWrapper,
        #       it will require all the parameters that they require, plus the ones specific
        #       to this class.

        # NOTE: get name of the probe from the config
        self.probe_name = config.get_name().split()[1]

        # NOTE: dummy extrude factor
        self.extrude_factor = 1.0

        self.printer = config.get_printer()

        # NOTE: Instantiate probe objects:
        #       -   "ProbeEndstopWrapper": Endstop wrapper that enables probe specific features.
        self.mcu_probe = probe_G38.ProbeEndstopWrapperG38(config)
        #       -   "mcu_probe_name": Readable name for the probe/endstop.
        self.mcu_probe_name ='probe_'+self.probe_name
        #       -   "PrinterProbe"/"PrinterProbeMux": Object registering the GCODE commands for probing.
        self.probe = PrinterProbeMux(config=config,
                                     mcu_probe=self.mcu_probe,
                                     mcu_probe_name=self.mcu_probe_name)

        # NOTE: register "mcu_probe" for endstop querying.
        self.mcu_probe.register_query_endstop(name=self.mcu_probe_name, config=config)

        # NOTE: save original probing config logic.
        #       This logic is used at "_home_cmd.send()" in "mcu.py"
        #       to make the low-level "endstop_home" MCU command.
        # self.invert_config = self.probe._invert

        # NOTE: not setup by "load_config", not needed either.
        #self.probe_name = config.get_name().split()[1]

        # NOTE: Override some things from the PrinterProbe init.
        # NOTE: They are no longer needed.
        #self.probe_pos = config.getfloat('endstop_position', self.probe.speed)
        #self.probe.z_position = self.probe_pos

        # NOTE: configure whether te move will be in absolute or relative coordinates
        #self.absolute_coord = config.getboolean('absolute_coord', True)

        # NOTE: Dummy position vector, populated later.
        self.last_position = [None, None, None, None]

        # NOTE: default probing speed
        self.speed = 100

        # NOTE: recovery stuff
        self.recovery_time = config.getfloat('recovery_time', 0.4, minval=0.)

        # NOTE: Register commands
        self.gcode = self.printer.lookup_object('gcode')

        # NOTE: From LinuxCNC: https://linuxcnc.org/docs/2.6/html/gcode/gcode.html
        #       - G38.2 - Probe toward workpiece, stop on contact, signal error if failure.
        self.gcode.register_mux_command("MULTIPROBE2", "PROBE_NAME",
                                        self.probe_name, self.cmd_PROBE_G38_2,
                                        #when_not_ready=False,
                                        desc=self.cmd_PROBE_G38_2_help)
        #       - G38.3 - Probe toward workpiece, stop on contact.
        self.gcode.register_mux_command("MULTIPROBE3", "PROBE_NAME",
                                        self.probe_name, self.cmd_PROBE_G38_3,
                                        #when_not_ready=False,
                                        desc=self.cmd_PROBE_G38_3_help)
        #       - G38.4 - Probe away from workpiece, stop on loss of contact, signal error if failure.
        self.gcode.register_mux_command("MULTIPROBE4", "PROBE_NAME",
                                        self.probe_name, self.cmd_PROBE_G38_4,
                                        #when_not_ready=False,
                                        desc=self.cmd_PROBE_G38_4_help)
        #       - G38.5 - Probe away from workpiece, stop on loss of contact.
        self.gcode.register_mux_command("MULTIPROBE5", "PROBE_NAME",
                                        self.probe_name, self.cmd_PROBE_G38_5,
                                        #when_not_ready=False,
                                        desc=self.cmd_PROBE_G38_5_help)

        # Register regular G38.n commands.
        # First check if this is the first instance of a multi-probe object.
        if "G38.2" in self.gcode.ready_gcode_handlers:
            self.main_object = False
            logging.info("probeProbeG38multi: G38.2 already configured, skipping G38.n register_command.")
        else:
            self.main_object = True
            logging.info("probeProbeG38multi: G38.2 not yet configured, running G38.n register_command.")

            # NOTE: From LinuxCNC: https://linuxcnc.org/docs/2.6/html/gcode/gcode.html
            #       - G38.2 - Probe toward workpiece, stop on contact, signal error if failure.
            self.gcode.register_command("G38.2",
                                        self.cmd_MONOPROBE_G38_2,
                                        when_not_ready=False,
                                        desc=self.cmd_MONOPROBE_G38_2_help)
            #       - G38.3 - Probe toward workpiece, stop on contact.
            self.gcode.register_command("G38.3",
                                        self.cmd_MONOPROBE_G38_3,
                                        when_not_ready=False,
                                        desc=self.cmd_MONOPROBE_G38_3_help)
            #       - G38.4 - Probe away from workpiece, stop on loss of contact, signal error if failure.
            self.gcode.register_command("G38.4",
                                        self.cmd_MONOPROBE_G38_4,
                                        when_not_ready=False,
                                        desc=self.cmd_MONOPROBE_G38_4_help)
            #       - G38.5 - Probe away from workpiece, stop on loss of contact.
            self.gcode.register_command("G38.5",
                                        self.cmd_MONOPROBE_G38_5,
                                        when_not_ready=False,
                                        desc=self.cmd_MONOPROBE_G38_5_help)

    def get_active_probe(self):
        """Get the "active" probe from the "active" extruder.
        """
        # Get the extruder name.
        toolhead = self.printer.lookup_object('toolhead')
        extruder_name = toolhead.extruder.name

        # If no extruder is configured, the name will be a None value,
        # reflecting that the current extruder object is a "Dummy" object.
        if extruder_name is None:
            # In that case, the default probe will be the first one that
            # was configured.
            msg = f"probe_G38_multi: No extruder found with name '{toolhead.extruder.name}'. "
            msg += f"G38 will use the first configured probe, with name: {self.probe_name}"
            self.gcode.respond_info(msg)
            extruder_name = self.probe_name

        # Look for the active probe object, by the extruder name.
        # This will raise an error if no match is found (see "klippy.py").
        probe_object = self.printer.lookup_object(name='probe_G38_multi' + ' ' + extruder_name)

        # Alternatively, all objects can be looked up.
        # probe_objects = self.printer.lookup_objects(module='probe_G38_multi')

        # Return the object, which is an instance of the ProbeG38multi class.
        return probe_object

    # MONOPROBE commands
    cmd_MONOPROBE_G38_5_help = "G38.5 Probe away from workpiece, stop on loss of contact."
    def cmd_MONOPROBE_G38_5(self, gcmd):
        # No error on failure, invert probe logic.
        self.cmd_PROBE_G38_2(gcmd, error_out=False, trigger_invert=False)

    cmd_MONOPROBE_G38_4_help = "G38.4 Probe away from workpiece, stop on loss of contact, signal error if failure."
    def cmd_MONOPROBE_G38_4(self, gcmd):
        # Error on failure, invert probe logic.
        self.cmd_PROBE_G38_2(gcmd, error_out=True, trigger_invert=False)

    cmd_MONOPROBE_G38_3_help = "G38.3 Probe toward workpiece, stop on contact."
    def cmd_MONOPROBE_G38_3(self, gcmd):
        # No error on failure, do not invert probe logic.
        self.cmd_PROBE_G38_2(gcmd, error_out=False, trigger_invert=True)

    cmd_MONOPROBE_G38_2_help = "G38.2 Probe toward workpiece, stop on contact, signal error if failure."
    def cmd_MONOPROBE_G38_2(self, gcmd, error_out=True, trigger_invert=True):
        # Error on failure, do not invert probe logic.

        # NOTE: Get the toolhead's last position.
        #       This will be updated below.
        toolhead = self.printer.lookup_object('toolhead')
        last_position = toolhead.get_position()

        # NOTE: get the name of the active extruder.
        extruder = toolhead.get_extruder()
        active_extruder_name = extruder.name

        # NOTE: configure whether te move will be in absolute
        #       or relative coordinates. Respect the G90/G91 setting.
        gcode_move = self.printer.lookup_object('gcode_move')
        absolute_coord = gcode_move.absolute_coord
        absolute_extrude = gcode_move.absolute_extrude

        # NOTE: also get the "base position". This is required to compute
        #       the absolute move, ¿relative to it? Weird...
        base_position = gcode_move.base_position

        # NOTE: Dummy objects for the G1 command parser
        speed_factor = 1

        # NOTE: probing axes list. This is populated with strings matching
        #       stepper names, coming from the axes involved in the probing
        #       move. For example, a probing move to X10,Y10 will have
        #       elements ["x", "y"]. These will then be matched to stepper
        #       names at the end of "probing_move" (see probing_move below
        #       and homing.py), to prevent raising "Probe triggered
        #       prior to movement" errors accidentally.
        probe_axes = []

        # NOTE: parse coordinates from a "regular" GCODE command.
        # NOTE: coordinate code parser copied from "cmd_G1" at "gcode_move.py".
        params = gcmd.get_command_parameters()
        try:
            # Parse axis coordinates
            for pos, axis in enumerate(list(toolhead.axis_map)[:-1]):
                if axis in params:
                    v = float(params[axis])
                    if not absolute_coord:
                        # value relative to position of last move
                        last_position[pos] += v
                    else:
                        # value relative to base coordinate position
                        last_position[pos] = v + base_position[pos]
                    # NOTE: register which axes are being probed
                    probe_axes.append(axis.lower())  # Append "X", "Y", or "Z".
            if 'E' in params:
                v = float(params['E']) * self.extrude_factor
                if not absolute_coord or not absolute_extrude:
                    # value relative to position of last move
                    last_position[toolhead.axis_count] += v
                else:
                    # value relative to base coordinate position
                    last_position[toolhead.axis_count] = v + base_position[toolhead.axis_count]
                # NOTE: register which axes are being probed
                probe_axes.append(active_extruder_name)  # Append "extruderN"

            # Parse feedrate
            speed = self.speed  # Default
            if 'F' in params:
                gcode_speed = float(params['F'])
                if gcode_speed <= 0.:
                    raise gcmd.error("Invalid speed in '%s'"
                                     % (gcmd.get_commandline(),))
                speed = gcode_speed * speed_factor

        except ValueError as e:
            raise gcmd.error(f"ProbeG38: Unable to parse move {gcmd.get_commandline()} with exception: {str(e)}")

        # NOTE: "move_with_transform" is just "toolhead.move":
        # self.move_with_transform(self.last_position, self.speed)

        # TODO: should this go here? borrowed code from "smart_effector"
        if self.recovery_time:
            toolhead.dwell(self.recovery_time)

        # Get active probe object, which might be the current ProbeG38multi instance,
        # or an instance for another probe pin. It is based on the active extruder name.
        probe_object = self.get_active_probe()

        # Get the probe_g38 method, corresponding to the current probe_object/extruder.
        probe_g38 = probe_object.probe_g38

        # NOTE: my probe works!
        probe_g38(pos=last_position, speed=speed,
                  error_out=error_out, gcmd=gcmd,
                  trigger_invert=trigger_invert,
                  probe_axes=probe_axes)


    # MULTIPROBE commands, overrides the default methods from "probe_G38.py".
    # TODO: cleanup. Since adding "monoprobe" commands, this class became rather obfuscated.
    cmd_PROBE_G38_2_help = "G38.2-style probe toward workpiece, stop on contact, signal error if failure. Usage: MULTIPROBEn PROBE_NAME=configname [X=x] [Y=x] [Z=x] [E=x]"
    cmd_PROBE_G38_3_help = "G38.3-style probe toward workpiece, stop on contact. Usage: MULTIPROBEn PROBE_NAME=configname [X=x] [Y=x] [Z=x] [E=x]"
    cmd_PROBE_G38_4_help = "G38.4-style probe away from workpiece, stop on loss of contact, signal error if failure. Usage: MULTIPROBEn PROBE_NAME=configname [X=x] [Y=x] [Z=x] [E=x]"
    cmd_PROBE_G38_5_help = "G38.5-style probe away from workpiece, stop on loss of contact. Usage: MULTIPROBEn PROBE_NAME=configname [X=x] [Y=x] [Z=x] [E=x]"
    def cmd_PROBE_G38_2(self, gcmd, error_out=True, trigger_invert=True):
        # Error on failure, do not invert probe logic.

        # NOTE: Get the toolhead's last position.
        #       This will be updated below.
        toolhead = self.printer.lookup_object('toolhead')
        self.last_position = toolhead.get_position()

        # NOTE: get the name of the active extruder.
        extruder = toolhead.get_extruder()
        active_extruder_name = extruder.name

        # NOTE: configure whether te move will be in absolute
        #       or relative coordinates. Respect the G90/G91 setting.
        gcode_move = self.printer.lookup_object('gcode_move')
        self.absolute_coord = gcode_move.absolute_coord
        self.absolute_extrude = gcode_move.absolute_extrude

        # NOTE: also get the "base position". This is required to compute
        #       the absolute move, ¿relative to it? Weird...
        base_position = gcode_move.base_position

        # NOTE: Dummy objects for the G1 command parser
        self.speed_factor = 1

        # NOTE: probing axes list. This is populated with strings matching
        #       stepper names, coming from the axes involved in the probing
        #       move. For example, a probing move to X10,Y10 will have
        #       elements ["x", "y"]. These will then be matched to stepper
        #       names at the end of "probing_move" (see probing_move below
        #       and homing.py), to prevent raising "Probe triggered
        #       prior to movement" errors accidentally.
        probe_axes = []

        # NOTE: parse coordinates from a "MUX" command.
        try:
            for pos, axis in enumerate(list(toolhead.axis_map)[:-1]):
                # NOTE: "pos" is 0, 1, 2.
                # NOTE: "axis" is X, Y, Z.
                coord = gcmd.get_float(axis, None)
                if coord is not None:
                    v = float(coord)
                    if not self.absolute_coord:
                        # value relative to position of last move
                        self.last_position[pos] += v
                    else:
                        # value relative to base coordinate position
                        self.last_position[pos] = v + base_position[pos]
                    # NOTE: register which axes are being probed
                    probe_axes.append(axis.lower())  # Append "X", "Y", or "Z".

            coord = gcmd.get_float('E', None)
            if coord is not None:
                v = float(coord) * self.extrude_factor
                if not self.absolute_coord or not self.absolute_extrude:
                    # value relative to position of last move
                    self.last_position[toolhead.axis_count] += v
                else:
                    # value relative to base coordinate position
                    self.last_position[toolhead.axis_count] = v + base_position[toolhead.axis_count]
                # NOTE: register which axes are being probed
                probe_axes.append(active_extruder_name)  # Append "extruderN"

            # Parse feedrate
            feed = gcmd.get_float('F', None)
            if feed is not None:
                gcode_speed = float(feed)
                if gcode_speed <= 0.:
                    raise gcmd.error("Invalid speed in '%s'"
                                     % (gcmd.get_commandline(),))
                self.speed = gcode_speed * self.speed_factor

        except ValueError as e:
            raise gcmd.error(f"ProbeG38: Unable to parse move {gcmd.get_commandline()} with exception: {str(e)}")

        # NOTE: "move_with_transform" is just "toolhead.move":
        # self.move_with_transform(self.last_position, self.speed)

        # TODO: should this go here?
        if self.recovery_time:
            toolhead.dwell(self.recovery_time)

        # NOTE: have a look at the methods in ProbeG38 at "probe_G38.py" for details.
        self.probe_g38(pos=self.last_position, speed=self.speed,
                       error_out=error_out, gcmd=gcmd,
                       trigger_invert=trigger_invert,
                       probe_axes=probe_axes)

class PrinterProbeMux(probe.PrinterProbe):
    def __init__(self, config, mcu_probe, mcu_probe_name='probe'):
        """
        This class inherits methods from "probe.PrinterProbe",
        but the commands are setup as "Mux" instead of "regular"
        GCODE commands. This is required to setup multiple probes.

        config: ?
        mcu_probe: this is of "ProbeEndstopWrapper" class, which is a wrapper for "MCU_endstop".
        """

        # TODO: check if this init can be stripped down.
        self.printer = config.get_printer()
        self.name = config.get_name()
        self.mcu_probe_name=mcu_probe_name
        self.mcu_probe = mcu_probe
        self.speed = config.getfloat('speed', 5.0, above=0.)
        self.lift_speed = config.getfloat('lift_speed', self.speed, above=0.)
        self.x_offset = config.getfloat('x_offset', 0.)
        self.y_offset = config.getfloat('y_offset', 0.)
        self.z_offset = config.getfloat('z_offset')
        self.probe_calibrate_z = 0.
        self.multi_probe_pending = False
        self.last_state = False
        self.last_z_result = 0.
        self.gcode_move = self.printer.load_object(config, "gcode_move")

        # Infer Z position to move to during a probe
        if config.has_section('stepper_z'):
            zconfig = config.getsection('stepper_z')
            self.z_position = zconfig.getfloat('position_min', 0.,
                                               note_valid=False)
        else:
            pconfig = config.getsection('printer')
            self.z_position = pconfig.getfloat('minimum_z_position', 0.,
                                               note_valid=False)

        # Multi-sample support (for improved accuracy)
        self.sample_count = config.getint('samples', 1, minval=1)
        self.sample_retract_dist = config.getfloat('sample_retract_dist', 2.,
                                                   above=0.)
        atypes = {'median': 'median', 'average': 'average'}
        self.samples_result = config.getchoice('samples_result', atypes,
                                               'average')
        self.samples_tolerance = config.getfloat('samples_tolerance', 0.100,
                                                 minval=0.)
        self.samples_retries = config.getint('samples_tolerance_retries', 0,
                                             minval=0)

        # Register z_virtual_endstop pin
        # TODO: study this to implement probing on any direction.
        self.printer.lookup_object('pins').register_chip(self.mcu_probe_name, self)

        # Register homing event handlers
        # TODO: these will not be triggered by extra toolheads, consider updating.
        self.printer.register_event_handler("homing:homing_move_begin",
                                            self._handle_homing_move_begin)
        self.printer.register_event_handler("homing:homing_move_end",
                                            self._handle_homing_move_end)
        self.printer.register_event_handler("homing:home_rails_begin",
                                            self._handle_home_rails_begin)
        self.printer.register_event_handler("homing:home_rails_end",
                                            self._handle_home_rails_end)
        self.printer.register_event_handler("gcode:command_error",
                                            self._handle_command_error)

        # Register PROBE/QUERY_PROBE commands
        self.gcode = self.printer.lookup_object('gcode')

        self.gcode.register_mux_command('PROBE', 'PROBE_NAME',
                                        self.mcu_probe_name,
                                        self.cmd_PROBE,
                                        desc=self.cmd_PROBE_help)

        self.gcode.register_mux_command('QUERY_PROBE', 'PROBE_NAME',
                                        self.mcu_probe_name,
                                        self.cmd_QUERY_PROBE,
                                        desc=self.cmd_QUERY_PROBE_help)

        self.gcode.register_mux_command('PROBE_CALIBRATE', 'PROBE_NAME',
                                        self.mcu_probe_name,
                                        self.cmd_PROBE_CALIBRATE,
                                        desc=self.cmd_PROBE_CALIBRATE_help)

        self.gcode.register_mux_command('PROBE_ACCURACY', 'PROBE_NAME',
                                        self.mcu_probe_name,
                                        self.cmd_PROBE_ACCURACY,
                                        desc=self.cmd_PROBE_ACCURACY_help)

        self.gcode.register_mux_command('Z_OFFSET_APPLY_PROBE', 'PROBE_NAME',
                                        self.mcu_probe_name,
                                        self.cmd_Z_OFFSET_APPLY_PROBE,
                                        desc=self.cmd_Z_OFFSET_APPLY_PROBE_help)

def load_config_prefix(config):
    return ProbeG38multi(config)
