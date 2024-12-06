# Z-Probe support
#
# Copyright (C) 2017-2021  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

# Type checking without cyclic import error.
# See: https://stackoverflow.com/a/39757388
from __future__ import annotations
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from ..klippy import Printer
    from ..kinematics import PrinterExtruder, ExtruderStepper
    from ..configfile import ConfigWrapper
    from ..toolhead import ToolHead
    from ..gcode import GCodeDispatch, GCodeCommand
    from .homing import PrinterHoming
    from .gcode_move import GCodeMove
# pylint: disable=missing-class-docstring,missing-function-docstring,invalid-name,line-too-long,consider-using-f-string
# pylint: disable=logging-fstring-interpolation,logging-not-lazy,fixme

import logging
from .probe import ProbeCommandHelper, PrinterProbe, ProbeOffsetsHelper, ProbeSessionHelper, ProbeEndstopWrapper, HINT_TIMEOUT

# Main external probe interface
class PrinterProbeG38(PrinterProbe):
    """Subclass of the main PrinterProbe in 'probe.py', using ProbeEndstopWrapperG38 instead.
    """
    def __init__(self, config: ConfigWrapper, mcu_probe_name='probe'):
        self.printer = config.get_printer()
        self.mcu_probe_name = mcu_probe_name
        self.mcu_probe = ProbeEndstopWrapperG38(config, mcu_probe_name)
        if config.getboolean('define_probe_commands', False):
            logging.info(f"Defining the standard PROBE commands with probe '{self.mcu_probe_name}'.")
            self.cmd_helper = ProbeCommandHelper(config, self,
                                                 self.mcu_probe.query_endstop)
        else:
            logging.info(f"Skipped definition of standard PROBE commands with probe '{self.mcu_probe_name}'.")
        self.probe_offsets = ProbeOffsetsHelper(config)
        self.probe_session = ProbeSessionHelper(config, self.mcu_probe, mcu_probe_name)

# Endstop wrapper that enables probe specific features
class ProbeEndstopWrapperG38(ProbeEndstopWrapper):
    """Subclass of ProbeEndstopWrapper, implementing multi-axis probing.
    This object is the 'mcu_probe' object elsewhere.
    """
    def __init__(self, config: ConfigWrapper, mcu_probe_name: str = 'probe'):

        # Instantiate the base "ProbeEndstopWrapper" class, as usual.
        # The parent class only reads from the "config" the "pin" parameter,
        # it does not require a name for it.
        super().__init__(config, mcu_probe_name)

        # NOTE: The super method adds several key objects used here:
        #       - self.printer
        #       - self.mcu_probe_name
        #       - self.mcu_endstop

        # Register probe for endstop querying.
        self.query_registered = False
        self.register_query_endstop(name=self.mcu_probe_name, config=config)

        # NOTE: recovery stuff, see "probe_prepare" below. Not needed.
        self.recovery_time = config.getfloat('recovery_time', 0.4, minval=0.)

        # NOTE: Add XY steppers too, see "_handle_mcu_identify" below.
        self.printer.register_event_handler('klippy:mcu_identify',
                                            self._handle_mcu_identify)

    def register_query_endstop(self, name, config):
        """Function used in 'probe_G38_multi' to register the probe endstop for display."""
        if not self.query_registered:
            logging.info(f"Registering endstop '{name}' as a G38 probing endstop.")
            # NOTE: grabbed from "stepper.py" to support querying the probes.
            # Load the "query_endstops" module.
            query_endstops = self.printer.load_object(config, 'query_endstops')
            # Register the endstop there.
            # NOTE: "self.mcu_endstop" was setup by "super" during init.
            query_endstops.register_endstop(self.mcu_endstop, name)
            # Flag registry.
            self.query_registered = True
        else:
            logging.info("Probe endstop already registered.")

    # NOTE: Register XY steppers in the endstop too.
    #       The following includes Z steppers and
    #       extruder steppers.
    def _handle_mcu_identify(self):
        logging.info(f"ProbeEndstopWrapperG38: associating all steppers to probe endstop '{self.mcu_probe_name}'.")

        # NOTE: Register XYZ steppers.
        toolhead: ToolHead = self.printer.lookup_object('toolhead')

        kins = toolhead.kinematics
        for ax_set in list(kins):
            kin = toolhead.get_kinematics(ax_set)
            if kin is not None:
                # NOTE: "kin.get_steppers" returns all "PrinterStepper"/"MCU_stepper" objects in the kinematic.
                for stepper in kin.get_steppers():
                    # NOTE: The usual 'xyz' letters are used here, even if they don't match the kin's axis names (e.g. ABC).
                    if stepper.is_active_axis('x') or stepper.is_active_axis('y') or stepper.is_active_axis('z'):
                        # NOTE: The "add_stepper" method called here is ultimately
                        #       from the "TriggerDispatch" class in "mcu.py",
                        self.add_stepper(stepper)

        # NOTE: register steppers from all extruders.
        extruder_objs = self.printer.lookup_extruders()
        for extruder_obj in extruder_objs:
            # extruder_name = extruder_obj[0]
            extruder: PrinterExtruder = extruder_obj[1]                      # PrinterExtruder
            extruder_stepper: ExtruderStepper = extruder.extruder_stepper    # ExtruderStepper
            for stepper in extruder_stepper.rail.get_steppers():
                # NOTE: this requires the PrinterRail or MCU_stepper objects
                #       to have the "get_steppers" method. The original MCU_stepper
                #       object did not, but it has been patched at "stepper.py".
                self.add_stepper(stepper)

class ProbeG38:
    """
    ! WARNING EXPERIMENTAL

    This class registers G38 commands to probe in general directions.

    The module respects the coordinate system set in gcode_move (i.e. absolute or relative mode).

    From LinuxCNC: https://linuxcnc.org/docs/2.6/html/gcode/gcode.html
        - G38.2 - (True/True) probe toward workpiece, stop on contact, signal error if failure.
        - G38.3 - (True/False) probe toward workpiece, stop on contact.
        - G38.4 - (False/True) probe away from workpiece, stop on loss of contact, signal error if failure.
        - G38.5 - (False/False) probe away from workpiece, stop on loss of contact.

    This feature relies on a great patch for the HomingMove class at "homing.py",
    and small patches in the ToolHead class at "toolhead.py", which
    enable support for extruder homing/probing. These are, broadly:
      - Added logic for calculating the extruder's kin_spos/haltpos/trigpos/etc.
      - Added logic to handle the active extruder in "check_no_movement".
      - Added "set_position_e" to the toolhead.
    """
    def __init__(self, config: ConfigWrapper, mcu_probe_name: str = 'probe'):
        # NOTE: Because the "config" is passed to PrinterProbe and ProbeEndstopWrapper,
        #       it will require all the parameters that they require, plus the ones specific
        #       to this class.
        self.mcu_probe_name = mcu_probe_name
        self.probe = self.setup_probe(config)
        self.printer: ConfigWrapper = config.get_printer()

        # NOTE: dummy extrude factor
        self.extrude_factor = 1.0

        # Dummy objects, replaced when "cmd_PROBE_G38_2" executes, 
        # with the current values stored in "gcode_move.py".
        self.absolute_coord: bool = None
        self.absolute_extrude: bool = None
        self.base_position: list = None

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

        # NOTE: Dummy position vector, overriden later.
        self.last_position = [None, None, None, None]

        # NOTE: recovery stuff
        self.recovery_time = config.getfloat('recovery_time', 0.4, minval=0.)

        # NOTE: Register commands
        self.gcode: GCodeDispatch = self.printer.lookup_object('gcode')
        self.register_commands()

        # NOTE: Get the proper ToolHead object.
        self.toolhead: ToolHead = None
        self.gcode_move: GCodeMove = None
        self.printer.register_event_handler('klippy:mcu_identify',
                                            self._handle_mcu_identify)

    def setup_probe(self, config):
        """"Instantiate PrinterProbeG38 object.
        Registers the commands for regular probing.
        """
        logging.info(f"Configuring G38.n commands for probe '{self.mcu_probe_name}'.")
        return PrinterProbeG38(config=config, mcu_probe_name=self.mcu_probe_name)

    def register_commands(self):
        """Register CNC-style probing commands."""
        # NOTE: From LinuxCNC: https://linuxcnc.org/docs/2.6/html/gcode/gcode.html
        #       - G38.2 - Probe toward workpiece, stop on contact, signal error if failure.
        self.gcode.register_command("G38.2",
                                    self.cmd_PROBE_G38_2,
                                    when_not_ready=False,
                                    desc=self.cmd_PROBE_G38_2_help)
        #       - G38.3 - Probe toward workpiece, stop on contact.
        self.gcode.register_command("G38.3",
                                    self.cmd_PROBE_G38_3,
                                    when_not_ready=False,
                                    desc=self.cmd_PROBE_G38_3_help)
        #       - G38.4 - Probe away from workpiece, stop on loss of contact, signal error if failure.
        self.gcode.register_command("G38.4",
                                    self.cmd_PROBE_G38_4,
                                    when_not_ready=False,
                                    desc=self.cmd_PROBE_G38_4_help)
        #       - G38.5 - Probe away from workpiece, stop on loss of contact.
        self.gcode.register_command("G38.5",
                                    self.cmd_PROBE_G38_5,
                                    when_not_ready=False,
                                    desc=self.cmd_PROBE_G38_5_help)

    def _handle_mcu_identify(self):
        # NOTE: Get the proper ToolHead object.
        self.toolhead: ToolHead = self.printer.lookup_object('toolhead')
        self.gcode_move: GCodeMove = self.printer.lookup_object("gcode_move")

    # Probe command variants
    cmd_PROBE_G38_5_help = "G38.5 Probe away from workpiece, stop on loss of contact."
    def cmd_PROBE_G38_5(self, gcmd):
        # No error on failure, invert probe logic.
        self.cmd_PROBE_G38_2(gcmd, error_out=False, trigger_invert=False)

    cmd_PROBE_G38_4_help = "G38.4 Probe away from workpiece, stop on loss of contact, signal error if failure."
    def cmd_PROBE_G38_4(self, gcmd):
        # Error on failure, invert probe logic.
        self.cmd_PROBE_G38_2(gcmd, error_out=True, trigger_invert=False)

    cmd_PROBE_G38_3_help = "G38.3 Probe toward workpiece, stop on contact."
    def cmd_PROBE_G38_3(self, gcmd):
        # No error on failure, do not invert probe logic.
        self.cmd_PROBE_G38_2(gcmd, error_out=False, trigger_invert=True)

    # Main probe command
    cmd_PROBE_G38_2_help = "G38.2 Probe toward workpiece, stop on contact, signal error if failure."
    def cmd_PROBE_G38_2(self, gcmd: GCodeCommand, error_out=True, trigger_invert=True):
        # Error on failure, do not invert probe logic.

        # NOTE: Get the toolhead's last position.
        #       This will be updated below.
        self.last_position = self.toolhead.get_position()

        # NOTE: get the name of the active extruder.
        extruder = self.toolhead.get_extruder()
        active_extruder_name = extruder.name

        # NOTE: configure whether te move will be in absolute
        #       or relative coordinates. Respect the G90/G91 setting.
        self.absolute_coord = self.gcode_move.absolute_coord
        self.absolute_extrude = self.gcode_move.absolute_extrude

        # NOTE: also get the "base position". This is required to compute
        #       the absolute move, ¿relative to it? Weird...
        self.base_position = self.gcode_move.base_position

        # NOTE: probing axes list. This is populated with strings matching
        #       stepper names, coming from the axes involved in the probing
        #       move. For example, a probing move to X10,Y10 will have
        #       elements ["x", "y"]. These will then be matched to stepper
        #       names at the end of "probing_move" (see probing_move below
        #       and homing.py), to prevent raising "Probe triggered
        #       prior to movement" errors accidentally.
        probe_axes = []

        # NOTE: coordinate code parser copied from "cmd_G1" at "gcode_move.py".
        params = gcmd.get_command_parameters()
        try:
            # Parse XYZ(ABC) axis move coordinates.
            for pos, axis in enumerate(list(self.toolhead.axis_map)[:-1]):
                if axis in params:
                    v = float(params[axis])
                    if not self.absolute_coord:
                        # Value relative to position of last move.
                        # Increment last position.
                        self.last_position[pos] += v
                    else:
                        # Absolute value, offset by base coordinate position.
                        # Overwrite last position.
                        self.last_position[pos] = v + self.base_position[pos]
                    # NOTE: register which axes are being probed
                    probe_axes.append(axis.lower())  # Append "X", "Y", or "Z".
            if 'E' in params:
                v = float(params['E']) * self.extrude_factor
                if not self.absolute_coord or not self.absolute_extrude:
                    # value relative to position of last move
                    self.last_position[self.toolhead.axis_count] += v
                else:
                    # value relative to base coordinate position
                    self.last_position[self.toolhead.axis_count] = v + self.base_position[self.toolhead.axis_count]
                # NOTE: register which axes are being probed
                probe_axes.append(active_extruder_name)  # Append "extruderN"

            # Parse feedrate
            speed = self.gcode_move.speed  # Default to the main speed (with speed factor applied).
            if 'F' in params:
                gcode_speed = float(params['F'])
                if gcode_speed <= 0.:
                    raise gcmd.error("Invalid speed in '%s'"
                                     % (gcmd.get_commandline(),))
                speed = gcode_speed * self.gcode_move.speed_factor

        except ValueError as e:
            raise gcmd.error(f"ProbeG38: Unable to parse move {gcmd.get_commandline()} with exception: {str(e)}")

        # NOTE: "move_with_transform" is just "toolhead.move":
        # self.move_with_transform(self.last_position, self.gcode_move.speed)

        # TODO: should this go here? borrowed code from "smart_effector"
        if self.recovery_time:
            self.toolhead.dwell(self.recovery_time)

        # NOTE: my probe works!
        self.probe_g38(pos=self.last_position, speed=speed,
                       error_out=error_out, gcmd=gcmd,
                       trigger_invert=trigger_invert,
                       probe_axes=probe_axes)

    def probe_g38(self, pos, speed, error_out, gcmd: GCodeCommand, trigger_invert, probe_axes=None):
        logging.info("probe_g38 probing with axes: " + str(probe_axes))

        # TODO: rethink if "homing" the machine is neccessary for probing.
        # curtime = self.printer.get_reactor().monotonic()
        # if 'z' not in toolhead.get_status(curtime)['homed_axes']:
        #     raise self.printer.command_error("Must home before probe")

        phoming: PrinterHoming = self.printer.lookup_object('homing')

        try:
            # NOTE: This probe method uses "phoming.probing_move",
            #       passing it "mcu_probe" which is an instance of
            #       "ProbeEndstopWrapper", a wrapper for the probe's
            #       MCU_endstop object. There is also "phoming.manual_home",
            #       which is similar but less convenient.
            # NOTE: The method is passed "pos", which is the target
            #       XYZE coordinates for the probing move (see notes
            #       above, and the "cmd_PROBE_G38_2" method).
            # NOTE: I had to add a "check_triggered" argument to
            #       "probing_move" for G38.3 to work properly.
            # NOTE: This "epos" is "trigpos" from the "homing_move" method.
            epos = phoming.probing_move(mcu_probe=self.probe.mcu_probe,
                                        pos=pos,
                                        speed=speed,
                                        check_triggered=error_out,
                                        # NOTE: new argument to probing_move.
                                        triggered=trigger_invert,
                                        # NOTE: new argument to probing_move.
                                        probe_axes=probe_axes)

        except self.printer.command_error as e:
            # NOTE: the "fail" logic of the G38 gcode could be
            #       based on this behaviour.
            reason = str(e)

            # NOTE: to respect the original logic, only "timeout" errors
            #       can be ignored. Else, the error should be logged with
            #       the "command_error" method, as always.
            if "Timeout during endstop homing" in reason:
                reason += HINT_TIMEOUT
                if error_out:
                    # NOTE: log the error as usual if it was requested.
                    raise self.printer.command_error(reason)
                else:
                    # NOTE: log as a "gcmd response"
                    gcmd.respond_info("G38 timeout without error, reason: " + reason)
            else:
                # NOTE: log the error as usual if it is was not a timeout error.
                raise self.printer.command_error(reason)

        # The toolhead's position was set to haltpos in "homing.py" after probing.
        haltpos = self.toolhead.get_position()
        status_prefix = "probe trigger"
        if haltpos == pos:
            # If "haltpos" and "target pos" are equal, then the move was not interrupted,
            # and no probe was triggered during the move.
            status_prefix = "probe ended without trigger"

        logging.info(f"probe_g38 probe ended with status: {status_prefix}")

        if self.toolhead.axis_count == 3:
            self.gcode.respond_info(status_prefix + " at x=%.3f y=%.3f z=%.3f e=%.3f" % tuple(epos))
        elif self.toolhead.axis_count == 6:
            self.gcode.respond_info(status_prefix + " at x=%.3f y=%.3f z=%.3f a=%.3f b=%.3f c=%.3f e=%.3f"
                                    % tuple(epos))
        else:
            # Get Current Position
            msg = " ".join([k.lower() + "=" + "%.3f" % haltpos[v] for k, v in self.toolhead.axis_map.items() ])
            self.gcode.respond_info(status_prefix + " at " + msg)
            # raise self.printer.command_error(f"Can't respond with info for toolhead.axis_count={toolhead.axis_count}")

        return epos[:-1]


def load_config(config):
    # TODO: Consider registering the PrinterProbe object as 'probe' in the printer.
    #       This would enable our 'probe' to be use by bed_mesh.
    #       For example in bltouch the load_config function does this:
    #           blt = BLTouchEndstopWrapper(config)
    #           config.get_printer().add_object('probe', probe.PrinterProbe(config, blt))
    #       I'd need to figure out the equivalent of BLTouchEndstopWrapper to pass.
    return ProbeG38(config)
