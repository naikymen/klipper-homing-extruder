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
from .probe import ProbeCommandHelper, PrinterProbe, ProbeOffsetsHelper, ProbeSessionHelper
from .probe_G38 import ProbeG38, ProbeEndstopWrapperG38

class ProbeG38multi(ProbeG38):
    """
    ! WARNING EXPERIMENTAL

    This class registers G38 commands to probe in general directions, and supports multiple probe endstops.

    The module respects the coordinate system set in gcode_move (i.e. absolute or relative mode).

    From LinuxCNC: https://linuxcnc.org/docs/2.6/html/gcode/gcode.html
        - G38.2 - (True/True) probe toward workpiece, stop on contact, signal error if failure.
        - G38.3 - (True/False) probe toward workpiece, stop on contact.
        - G38.4 - (False/True) probe away from workpiece, stop on loss of contact, signal error if failure.
        - G38.5 - (False/False) probe away from workpiece, stop on loss of contact.

    In order to support multiple endstops, new pseudo-GCODE commands were added to select the probe.
    These are the "MULTIPROBE" commands below.

    This class also registers the regular G38 commands, which will use the probe endstop whose name
    matches the name of the active extruder.

    This feature relies on a great patch for the HomingMove class at "homing.py",
    and small patches in the ToolHead class at "toolhead.py", which
    enable support for extruder homing/probing. These are, broadly:
      - Added logic for calculating the extruder's kin_spos/haltpos/trigpos/etc.
      - Added logic to handle the active extruder in "check_no_movement".
      - Added "set_position_e" to the toolhead.
    """
    def __init__(self, config):
        # NOTE: because the "config" is passed to PrinterProbe and ProbeEndstopWrapper,
        #       it will require all the parameters that they require, plus the ones specific
        #       to this class.

        # NOTE: Get name of the probe from the config.
        #       E.g. gets "p200_probe" from a config section titled "[probe_G38_multi p200_probe]".
        self.probe_name = config.get_name().split()[1]

        # NOTE: Readable name for the probe/endstop.
        self.mcu_probe_name = 'probe_' + self.probe_name

        # NOTE: Call the init method in ProbeG38.
        #       It is important that certain methods are overridden below for this to work:
        #       "setup_probe" and "register_commands".
        super().__init__(config, self.mcu_probe_name)

    def setup_probe(self, config):
        """"Override of the 'setup_probe' method in ProbeG38.
        Instantiate PrinterProbeG38Mux object. Registers the commands for regular probing.
        """
        logging.info(f"Configuring MULTIPROBE and G38.n commands for probe '{self.probe_name}'.")
        return PrinterProbeG38Mux(config=config, mcu_probe_name=self.mcu_probe_name)

    def register_commands(self):
        """Override the method from the parent ProbeG38 class.
        Register CNC-style probing commands."""
        # NOTE: From LinuxCNC: https://linuxcnc.org/docs/2.6/html/gcode/gcode.html
        #       - G38.2 - Probe toward workpiece, stop on contact, signal error if failure.
        self.gcode.register_mux_command("MULTIPROBE2", "PROBE_NAME",
                                        self.probe_name, self.cmd_MULTIPROBE_2,
                                        #when_not_ready=False,
                                        desc=self.cmd_MULTIPROBE_2_help)
        #       - G38.3 - Probe toward workpiece, stop on contact.
        self.gcode.register_mux_command("MULTIPROBE3", "PROBE_NAME",
                                        self.probe_name, self.cmd_MULTIPROBE_3,
                                        #when_not_ready=False,
                                        desc=self.cmd_MULTIPROBE_3_help)
        #       - G38.4 - Probe away from workpiece, stop on loss of contact, signal error if failure.
        self.gcode.register_mux_command("MULTIPROBE4", "PROBE_NAME",
                                        self.probe_name, self.cmd_MULTIPROBE_4,
                                        #when_not_ready=False,
                                        desc=self.cmd_MULTIPROBE_4_help)
        #       - G38.5 - Probe away from workpiece, stop on loss of contact.
        self.gcode.register_mux_command("MULTIPROBE5", "PROBE_NAME",
                                        self.probe_name, self.cmd_MULTIPROBE_5,
                                        #when_not_ready=False,
                                        desc=self.cmd_MULTIPROBE_5_help)

        # Register regular G38.n commands.
        # First check if this is the first instance of a multi-probe object.
        if "G38.2" in self.gcode.ready_gcode_handlers:
            self.main_object = False
            logging.info("ProbeG38multi: G38.2 already configured, skipping G38.n register_command.")
        else:
            self.main_object = True
            logging.info("ProbeG38multi: G38.2 not yet configured, running G38.n register_command.")
            # NOTE: Register the commands using the parent methods from ProbeG38.
            super().register_commands()

    def get_active_probe(self):
        """Get the "active" probe from the "active" extruder by name.
        This is used to default to the active extruder for regular G38 commands.
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

    # MULTIPROBE command variants.
    cmd_MULTIPROBE_5_help = "G38.5-style probe away from workpiece, stop on loss of contact. Usage: MULTIPROBE5 PROBE_NAME=<probe name> [X=x] [Y=y] [Z=z] [E=e]"
    def cmd_MULTIPROBE_5(self, gcmd):
        # No error on failure, invert probe logic.
        self.cmd_MULTIPROBE_2(gcmd, error_out=False, trigger_invert=False)

    cmd_MULTIPROBE_4_help = "G38.4-style probe away from workpiece, stop on loss of contact, signal error if failure. Usage: MULTIPROBE4 PROBE_NAME=<probe name> [X=x] [Y=y] [Z=z] [E=e]"
    def cmd_MULTIPROBE_4(self, gcmd):
        # Error on failure, invert probe logic.
        self.cmd_MULTIPROBE_2(gcmd, error_out=True, trigger_invert=False)

    cmd_MULTIPROBE_3_help = "G38.3-style probe toward workpiece, stop on contact. Usage: MULTIPROBE3 PROBE_NAME=<probe name> [X=x] [Y=y] [Z=z] [E=e]"
    def cmd_MULTIPROBE_3(self, gcmd):
        # No error on failure, do not invert probe logic.
        self.cmd_MULTIPROBE_2(gcmd, error_out=False, trigger_invert=True)

    cmd_MULTIPROBE_2_help = "G38.2-style probe toward workpiece, stop on contact, signal error if failure. Usage: MULTIPROBE2 PROBE_NAME=<probe name> [X=x] [Y=y] [Z=z] [E=e]"
    def cmd_MULTIPROBE_2(self, gcmd, error_out=True, trigger_invert=True):
        """Klipper style keyword-argument probing commands"""
        super().cmd_PROBE_G38_2(gcmd, error_out=error_out, trigger_invert=trigger_invert)

class ProbeCommandHelperMux(ProbeCommandHelper):
    """Subclass of ProbeCommandHelper, muxing all commands.
    Replacement of PrinterProbeMux."""
    def __init__(self, config, probe: PrinterProbeG38Mux, query_endstop=None):
        self.printer = config.get_printer()
        self.probe = probe
        self.query_endstop = query_endstop
        self.name = config.get_name()

        # Register PROBE/QUERY_PROBE commands
        self.gcode = self.printer.lookup_object('gcode')

        self.gcode.register_mux_command('PROBE', 'PROBE_NAME',
                                        self.probe.mcu_probe_name,
                                        self.cmd_PROBE,
                                        desc=self.cmd_PROBE_help)

        self.gcode.register_mux_command('QUERY_PROBE', 'PROBE_NAME',
                                        self.probe.mcu_probe_name,
                                        self.cmd_QUERY_PROBE,
                                        desc=self.cmd_QUERY_PROBE_help)

        self.gcode.register_mux_command('PROBE_CALIBRATE', 'PROBE_NAME',
                                        self.probe.mcu_probe_name,
                                        self.cmd_PROBE_CALIBRATE,
                                        desc=self.cmd_PROBE_CALIBRATE_help)

        self.gcode.register_mux_command('PROBE_ACCURACY', 'PROBE_NAME',
                                        self.probe.mcu_probe_name,
                                        self.cmd_PROBE_ACCURACY,
                                        desc=self.cmd_PROBE_ACCURACY_help)

        self.gcode.register_mux_command('Z_OFFSET_APPLY_PROBE', 'PROBE_NAME',
                                        self.probe.mcu_probe_name,
                                        self.cmd_Z_OFFSET_APPLY_PROBE,
                                        desc=self.cmd_Z_OFFSET_APPLY_PROBE_help)

# Main external probe interface
class PrinterProbeG38Mux(PrinterProbe):
    """Subclass of the main PrinterProbe in 'probe.py'
    Using ProbeEndstopWrapperG38 and ProbeCommandHelperMux instead.
    """
    def __init__(self, config: ConfigWrapper, mcu_probe_name='probe'):
        self.printer = config.get_printer()
        self.mcu_probe_name = mcu_probe_name
        self.mcu_probe = ProbeEndstopWrapperG38(config, mcu_probe_name)
        self.cmd_helper = ProbeCommandHelperMux(config, self,
                                                self.mcu_probe.query_endstop)
        self.probe_offsets = ProbeOffsetsHelper(config)
        self.probe_session = ProbeSessionHelper(config, self.mcu_probe, mcu_probe_name)


def load_config_prefix(config):
    return ProbeG38multi(config)
