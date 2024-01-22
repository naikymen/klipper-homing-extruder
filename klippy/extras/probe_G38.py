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
    from ..configfile import ConfigWrapper
    from ..toolhead import ToolHead
    from ..gcode import GCodeDispatch, GCodeCommand
    from .homing import PrinterHoming
    from .gcode_move import GCodeMove

# TODO: check if this is useful.
# from . import manual_probe

import logging
import pins
from . import probe


class ProbeEndstopWrapperG38(probe.ProbeEndstopWrapper):
    def __init__(self, config: ConfigWrapper):
        
        # Instantiate the base "ProbeEndstopWrapper" class, as usual.
        # The parent class only reads from the "config" the "pin" parameter,
        # it does not require a name for it.
        super(ProbeEndstopWrapperG38, self).__init__(config)

        # NOTE: not really needed, its done already by "super()".
        self.printer = config.get_printer()

        # NOTE: recovery stuff, see "probe_prepare" below. Not needed.
        self.recovery_time = config.getfloat('recovery_time', 0.4, minval=0.)

        # NOTE: add XY steppers too, see "_handle_mcu_identify" below.
        self.printer.register_event_handler('klippy:mcu_identify',
                                            self._handle_mcu_identify)
        
    def register_query_endstop(self, name, config):
        # NOTE: grabbed from "stepper.py" to support querying the probes.
        # Load the "query_endstops" module.
        query_endstops = self.printer.load_object(config, 'query_endstops')
        # Register the endstop there.
        # NOTE: "self.mcu_endstop" was setup by "super" during init.
        query_endstops.register_endstop(self.mcu_endstop, name)

    # Overwrite only the "probe_prepare" method, to include the dwell.
    def probe_prepare(self, hmove):
        if self.multi == 'OFF' or self.multi == 'FIRST':
            self.lower_probe()
            if self.multi == 'FIRST':
                self.multi = 'ON'
            
        # NOTE: borrowed code from "smart_effector", trying to
        #       avoid the "Probe triggered prior to movement" error.
        if self.recovery_time:
            toolhead: ToolHead = self.printer.lookup_object('toolhead')
            toolhead.dwell(self.recovery_time)

    # NOTE: Register XY steppers in the endstop too.
    #       The following includes Z steppers and 
    #       extruder steppers.
    def _handle_mcu_identify(self):
        logging.info(f"\n\n" + "ProbeEndstopWrapperG38._handle_mcu_identify activated (XYZE axes)" + "\n\n")

        # NOTE: Register XYZ steppers.
        toolhead: ToolHead = self.printer.lookup_object('toolhead')
        kin = toolhead.get_kinematics()
        if kin is not None:
            for stepper in kin.get_steppers():
                # NOTE: get_steppers returns all "PrinterStepper"/"MCU_stepper" objects in the kinematic.
                if stepper.is_active_axis('x') or stepper.is_active_axis('y') or stepper.is_active_axis('z'):
                    self.add_stepper(stepper)
                
        # NOTE: Register ABC steppers too.
        kin_abc = toolhead.get_kinematics_abc()
        if kin_abc is not None:
            for stepper in kin_abc.get_steppers():
                # NOTE: get_steppers returns all "PrinterStepper"/"MCU_stepper" objects in the kinematic.
                if stepper.is_active_axis('x') or stepper.is_active_axis('y') or stepper.is_active_axis('z'):
                    self.add_stepper(stepper)
        
        # NOTE: register steppers from all extruders.
        extruder_objs = self.printer.lookup_extruders()
        for extruder_obj in extruder_objs:
            extruder_name = extruder_obj[0]
            extruder = extruder_obj[1]                      # PrinterExtruder
            extruder_stepper = extruder.extruder_stepper    # ExtruderStepper
            for stepper in extruder_stepper.rail.get_steppers():
                # NOTE: this requires the PrinterRail or MCU_stepper objects 
                #       to have the "get_steppers" method. The original MCU_stepper
                #       object did not, but it has been patched at "stepper.py".
                self.add_stepper(stepper)


class ProbeG38:
    """
    ! WARNING EXPERIMENTAL

    This class registers G38 commands to probe in general directions.

    From LinuxCNC: https://linuxcnc.org/docs/2.6/html/gcode/gcode.html
        - G38.2 - (True/True) probe toward workpiece, stop on contact, signal error if failure.
        - G38.3 - (True/False) probe toward workpiece, stop on contact.
        - G38.4 - (False/True) probe away from workpiece, stop on loss of contact, signal error if failure.
        - G38.5 - (False/False) probe away from workpiece, stop on loss of contact.
    
    This feature relies on a great patch for the HomingMove class at "homing.py",
    and small patches in the ToolHead class at "toolhead.py", which
    enable support for extruder homing/probing. These are mainly:
      - Added logic for calculating the extruder's kin_spos/haltpos/trigpos/etc.
      - Added logic to handle the active extruder in "check_no_movement".
      - Added "set_position_e" to the toolhead.

    """
    def __init__(self, config: ConfigWrapper, mcu_probe_name='probe'):
        # NOTE: because the "config" is passed to PrinterProbe and ProbeEndstopWrapper,
        #       it will require all the parameters that they require, plus the ones specific
        #       to this class.

        # NOTE: instantiate:
        #       -   "ProbeEndstopWrapper": Endstop wrapper that enables probe specific features.
        #       -   "PrinterProbe": ?
        #self.probe = probe.PrinterProbe(config, probe.ProbeEndstopWrapper(config))
        self.mcu_probe_name=mcu_probe_name
        self.probe = probe.PrinterProbe(config=config, mcu_probe=ProbeEndstopWrapperG38(config))
        self.printer = config.get_printer()
        
        # NOTE: dummy extrude factor
        self.extrude_factor = 1.0

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

        # NOTE: default probing speed
        self.speed = 100

        # NOTE: recovery stuff
        self.recovery_time = config.getfloat('recovery_time', 0.4, minval=0.)

        # NOTE: Register commands
        self.gcode: GCodeDispatch = self.printer.lookup_object('gcode')
        
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
        toolhead: ToolHead = self.printer.lookup_object('toolhead')
        self.last_position = toolhead.get_position()

        # NOTE: get the name of the active extruder.
        extruder = toolhead.get_extruder()
        active_extruder_name = extruder.name

        # NOTE: configure whether te move will be in absolute 
        #       or relative coordinates. Respect the G90/G91 setting.
        gcode_move: GCodeMove = self.printer.lookup_object('gcode_move')
        self.absolute_coord = gcode_move.absolute_coord
        self.absolute_extrude = gcode_move.absolute_extrude

        # NOTE: also get the "base position". This is required to compute
        #       the absolute move, ¿relative to it? Weird...
        self.base_position = gcode_move.base_position

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

        # NOTE: coordinate code parser copied from "cmd_G1" at "gcode_move.py".
        params = gcmd.get_command_parameters()
        try:
            # Parse XYZ(ABC) axis move coordinates.
            for pos, axis in enumerate(list(toolhead.axis_map)[:-1]):
                if axis in params:
                    v = float(params[axis])
                    if not self.absolute_coord:
                        # value relative to position of last move
                        self.last_position[pos] += v
                    else:
                        # value relative to base coordinate position
                        self.last_position[pos] = v + self.base_position[pos]
                    # NOTE: register which axes are being probed
                    probe_axes.append(axis.lower())  # Append "X", "Y", or "Z".
            if 'E' in params:
                v = float(params['E']) * self.extrude_factor
                if not self.absolute_coord or not self.absolute_extrude:
                    # value relative to position of last move
                    self.last_position[toolhead.axis_count] += v
                else:
                    # value relative to base coordinate position
                    self.last_position[toolhead.axis_count] = v + self.base_position[toolhead.axis_count]
                # NOTE: register which axes are being probed
                probe_axes.append(active_extruder_name)  # Append "extruderN"
            
            # Parse feedrate
            if 'F' in params:
                gcode_speed = float(params['F'])
                if gcode_speed <= 0.:
                    raise gcmd.error("Invalid speed in '%s'"
                                     % (gcmd.get_commandline(),))
                self.speed = gcode_speed * self.speed_factor
        
        except ValueError as e:
            raise gcmd.error(f"ProbeG38: Unable to parse move {gcmd.get_commandline()} with exception: {str(e)}")
        
        # NOTE: "move_with_transform" is just "toolhead.move":
        # self.move_with_transform(self.last_position, self.speed)

        # TODO: should this go here? borrowed code from "smart_effector"
        if self.recovery_time:
            toolhead.dwell(self.recovery_time)
        
        # NOTE: my probe works!
        self.probe_g38(pos=self.last_position, speed=self.speed, 
                       error_out=error_out, gcmd=gcmd, 
                       trigger_invert=trigger_invert,
                       probe_axes=probe_axes)

    def probe_g38(self, pos, speed, error_out, gcmd: GCodeCommand, trigger_invert, probe_axes=None):
        # NOTE: code copied from "probe._probe".

        toolhead: ToolHead = self.printer.lookup_object('toolhead')

        # TODO: rethink if homing is neccessary for homing.
        # curtime = self.printer.get_reactor().monotonic()
        # if 'z' not in toolhead.get_status(curtime)['homed_axes']:
        #     raise self.printer.command_error("Must home before probe")
        
        phoming: PrinterHoming = self.printer.lookup_object('homing')

        # NOTE: This is no longer necessary, because I've passed
        #       the "pos" argument from "cmd_PROBE_G38_2".
        # pos = toolhead.get_position()

        # NOTE: This is also no longer necessary.
        # NOTE: "self.z_position" is equal to the "min_position"
        #       parameter from the "z_stepper" section.
        #       It is used to override the Z component of the 
        #       current toolhead position, probably to generate
        #       the target coordinates for the homing move.
        # pos[2] = self.z_position
        
        try:
            # NOTE: This probe method uses "phoming.probing_move",
            #       passing it "mcu_probe" which is an instance of 
            #       "ProbeEndstopWrapper", a wrapper for the probes'
            #       MCU_endstop object.
            # NOTE: This is in contrast to "phoming.manual_home",
            #       which additionally requires a toolhead object.
            #       It turns out that, if not provided, HomingMove
            #       will get the main toolhead by lookup and use it.
            # NOTE: the method is passed "pos", which is the target
            #       XYZE coordinates for the probing move (see notes 
            #       above, and the "cmd_PROBE_G38_2" method). 
            # NOTE: I had to add a "check_triggered" argument to 
            #       "probing_move" for G38.3 to work properly.
            logging.info(f"\n\n" + "probe_g38 probing with axes: " + str(probe_axes) + "\n\n")
            
            # NOTE: "epos" is "trigpos" from the "homing_move" method.
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
                reason += probe.HINT_TIMEOUT
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
        haltpos = toolhead.get_position()
        status_prefix = "probe trigger"
        if haltpos == pos:
            # If "haltpos" and "target pos" are equal, then the move was not interrupted,
            # and no probe was triggered during the move.
            status_prefix = "probe ended without trigger"
        
        logging.info("\n\n" + f"probe_g38 probe ended with status: {status_prefix}" + "\n\n")

        if toolhead.axis_count == 3:
            self.gcode.respond_info(status_prefix + " at x=%.3f y=%.3f z=%.3f e=%.3f" % tuple(epos))
        elif toolhead.axis_count == 6:
            self.gcode.respond_info(status_prefix + " at x=%.3f y=%.3f z=%.3f a=%.3f b=%.3f c=%.3f e=%.3f"
                                    % tuple(epos))
        else:
            # Get Current Position
            msg = " ".join([k.lower() + "=" + "%.3f" % haltpos[v] for k, v in toolhead.axis_map.items() ])
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
