# By naikymen and mdwasp
# Original idea at: https://discord.com/channels/627982902738681876/1046618170993160202/1046808809894588457
# Relevant issue: https://gitlab.com/pipettin-bot/pipettin-grbl/-/issues/47#note_1215525244
# Module distributed under the terms of the GNU GPL v3 licence.
#
#
# This class loads the stepper of the active extruder and performs homing on it.
# It is inspired by the manual_stepper module, and it requires a few changes in
# homing.py and toolhead.py to work: https://github.com/Klipper3d/klipper/pull/5950
#
# The module requires a modification in "extruder.py", which will create the extruder
# stepper from the PrinterRail class, instead of the PrinterStepper class, when an
# "endstop_pin" is defined in the extruder's config.
#
# A config section is required to activate it, and a command must be sent to use it.
# When activated, some additional parameters must be provided to each [extruder] section.
# For example configuration files, see:
#   config/configs-pipetting-bot/configs-mainsail/printer.cfg
#   config/configs-pipetting-bot/configs-mainsail/home_extruder.cfg

# pylint: disable=missing-class-docstring,missing-function-docstring,invalid-name,line-too-long,consider-using-f-string,multiple-imports,wrong-import-position
# pylint: disable=logging-fstring-interpolation,logging-not-lazy,fixme

# from collections import namedtuple
import logging

import stepper  # , chelper
# from toolhead import Move
from kinematics.extruder import PrinterExtruder, ExtruderStepper
from extras.homing import PrinterHoming
from gcode import GCodeDispatch
from toolhead import ToolHead
from klippy import Printer

class ExtruderHoming:
    """
    ! WARNING EXPERIMENTAL
    This class registers a command to home an extruder's stepper.
    This is made possible due to a change in the Extruder class,
    which can now add an endstop to the extruder stepper, if a
    config parameter is provided.

    It relies on a great patch for the HomingMove class at "homing.py",
    and small patches in the ToolHead class at "toolhead.py", which
    enable support for extruder homing/probing. These are mainly:
      - Added logic for calculating the extruder's kin_spos/haltpos/trigpos/etc.
      - Added "set_position_e" to the toolhead.

    A note for achaeologists:
    The "toolhead" object that used to be passed to the
    PrinterHoming.manual_home method was mostly "virtual".
    Many methods are defined here, but some of them call the
    methods of the actual toolhead. All of these methods are
    commented out below.
    See commit: 8eb3366b6ee1eb74c70a715db66152b13a2d4372
    """
    def __init__(self, config):
        self.printer: Printer = config.get_printer()
        self.extruder_name = config.get_name().split()[1]

        self.toolhead = None
        self.extruder = None
        self.gcmd = None
        self.th_orig_pos = None

        # To check if a "HOME_EXTRUDER" command has been received already.
        self.homing = False

        # NOTE: some parameters are loaded from the "extruder_homing" config section,
        #       and used by some of the move methods (not necesarily by drip_move).
        # self.velocity = config.getfloat('velocity', 5., above=0.)
        # self.accel = self.homing_accel = config.getfloat('accel', 0., minval=0.)

        # TODO: find out what this is. The same is used in manual_stepper.py
        #       No longer required, it used to be updated by sync_print_time.
        self.next_cmd_time = 0.

        # NOTE: The following command will become available with the syntax:
        #       "HOME_EXTRUDER EXTRUDER=extruder", the extruder name
        #        passed to the EXTRUDER argument might change.
        # Register commands
        self.gcode: GCodeDispatch = self.printer.lookup_object('gcode')
        self.gcode.register_mux_command('HOME_EXTRUDER', "EXTRUDER",
                                        self.extruder_name, self.cmd_HOME_EXTRUDER,
                                        desc=self.cmd_HOME_EXTRUDER_help)

        # Register active extruder homing command.
        # First check if this is the first instance of a multi-probe object.
        if "HOME_ACTIVE_EXTRUDER" in self.gcode.ready_gcode_handlers:
            self.main_object = False
            logging.info("ExtruderHoming: HOME_ACTIVE_EXTRUDER already configured, skipping HOME_ACTIVE_EXTRUDER register_command.")
        else:
            self.main_object = True
            logging.info("ExtruderHoming: HOME_ACTIVE_EXTRUDER not yet configured, running HOME_ACTIVE_EXTRUDER register_command.")

            self.gcode.register_command("HOME_ACTIVE_EXTRUDER",
                                        self.cmd_HOME_ACTIVE_EXTRUDER,
                                        when_not_ready=False,
                                        desc=self.cmd_HOME_ACTIVE_EXTRUDER_help)

        # Placeholders
        self.active_extruder: PrinterExtruder = None
        self.active_extruder_name: str = None
        self.extruder_trapq = None
        self.extruder_stepper = None
        self.rail = None
        self.stepper = None
        self.steppers = None
        self.homing_info = None

        logging.info("ExtruderHoming: init complete")

        # # NOTE: setup event handler to "finalize" the extruder trapq after
        # #       a drip move, but before the "flush_step_generation" call.
        # ffi_main, ffi_lib = chelper.get_ffi()
        # self.trapq_finalize_moves = ffi_lib.trapq_finalize_moves
        # self.printer.register_event_handler("toolhead:trapq_finalize_extruder_drip_moves",
        #                                     self.handle_drip_move_end)

    # # NOTE: This must only execute in the "right" context (i.e. during extruder homing
    # #       and not duting regular XYZ homing; at least until I test otherwise).
    # def handle_drip_move_end(self, never_time, extruder_name):
    #     if (self.homing is True) and (extruder_name == self.extruder_name):
    #         logging.info(f"{self.extruder_name} handle_drip_move_end: calling trapq_finalize_moves on '{extruder_name}'")
    #         self.trapq_finalize_moves(self.extruder_trapq, never_time)
    #     else:
    #         # NOTE: this will fire either on other instances of "ExtruderHoming",
    #         #       or also out of place, during homing of other axis.
    #         logging.info(f"{self.extruder_name} handle_drip_move_end: skipped out of context trapq_finalize_moves ")

    # NOTE: the "register_mux_command" above registered a "HOME_EXTRUDER"
    #       command, which will end up calling this method.
    #       The "help" string is usually defined along the method.
    cmd_HOME_EXTRUDER_help = "Home an extruder using an endstop. Only the active extruder can be homed."
    def cmd_HOME_EXTRUDER(self, gcmd):
        """
        Usage: HOME_EXTRUDER EXTRUDER=<extruder name>
        """

        # Get gcmd object, for later.
        self.gcmd = gcmd

        # NOTE: Borrowed from extruder.py
        self.extruder: PrinterExtruder = self.printer.lookup_object(self.extruder_name, None)  # PrinterExtruder
        if self.extruder is None or not isinstance(self.extruder, PrinterExtruder):
            raise self.printer.command_error(f"'{self.extruder_name}' is not a valid extruder.")

        # NOTE: Get the toolhead and its *current* extruder.
        self.toolhead: ToolHead = self.printer.lookup_object("toolhead")
        self.active_extruder: PrinterExtruder = self.toolhead.get_extruder()            # PrinterExtruder
        self.active_extruder_name = self.active_extruder.get_name()

        # NOTE: check if the active extruder is the one to be homed.
        if self.extruder_name != self.active_extruder_name:
            try:
                # NOTE: activate the requested extruder if necessary.
                self.extruder.cmd_ACTIVATE_EXTRUDER(gcmd=gcmd)
            except:
                raise gcmd.error("ExtruderHoming.cmd_HOME_EXTRUDER: " +
                                 f"{self.active_extruder_name} is active " +
                                 f"but homing {self.extruder_name} was requested. " +
                                 f"Could not activate {self.active_extruder_name}.")

        # NOTE: Get the active extruder's trapq.
        self.extruder_trapq = self.extruder.get_trapq()         # extruder trapq (from ffi)

        # NOTE: Get the steppers
        self.extruder_stepper: ExtruderStepper = self.extruder.extruder_stepper      # ExtruderStepper
        self.rail: stepper.PrinterRail = self.extruder_stepper.rail # PrinterRail
        self.stepper: stepper.MCU_stepper = self.extruder_stepper.stepper   # MCU_stepper
        self.steppers = [self.stepper]                              # [MCU_stepper]
        # NOTE: in the "ExtruderStepper" class, the "rail" and the "stepper"
        #       objects are _the same_ object.

        # Check that an enstop was configured.
        # NOTE: Checking that the stepper is a PrinterRail (instead of a PrinterStepper) should be enough for now.
        if not isinstance(self.rail, stepper.PrinterRail):
            raise gcmd.error("ExtruderHoming error: " +
                             f"Homing was not configured for extruder '{self.extruder_name}'. " +
                             "Add endstop parameters to the extruder's config section to enable homing.")

        # NOTE: Get the endstops from the extruder's PrinterRail.
        #       likely a list of tuples, each with an instance of
        #       MCU_endstop and a stepper name.
        #       See PrinterRail at stepper.py.
        endstops = self.rail.get_endstops()                 # [(mcu_endstop, name)]

        # NOTE: get a PrinterHoming class from extras.
        phoming: PrinterHoming = self.printer.lookup_object('homing')      # PrinterHoming

        # NOTE: Get original toolhead position
        self.th_orig_pos = self.toolhead.get_position()

        # NOTE: Get homing information, speed and move coordinate.
        self.homing_info = self.rail.get_homing_info()
        speed = self.homing_info.speed

        # NOTE: Use XYZ from the toolhead, and E from the config file + estimation.
        pos = self.th_orig_pos[:-1] + [self.get_movepos(self.homing_info)]

        # Get rail limits
        position_min, position_max = self.rail.get_range()

        # NOTE: Force extruder to a certain starting position.
        #       Originally 0.0, now position_max, which requires an
        #       endstop position of 0.0 to home in the right direction.
        if self.homing_info.positive_dir:
            # NOTE: pos[-1] is the endstop's position.
            e_startpos = position_min - pos[-1] * 0.1
        else:
            # NOTE: pos[-1] is the endstop's position.
            e_startpos = position_max + pos[-1] * 0.1
        startpos = self.th_orig_pos[:-1] + [e_startpos]
        self.toolhead.set_position(newpos=startpos,
                                   homing_axes=(self.toolhead.axis_count, ))

        # NOTE: flag homing start
        self.homing = True

        # NOTE: "manual_home" is defined in the PrinterHoming class (at homing.py).
        #       The method instantiates a "HomingMove" class by passing it the
        #       "endstops" and "toolhead" objects.
        #       The requried "endstop"s are from the extruder's PrinterRail object.
        #       In the "manual_stepper" object, the very "self" object is passed
        #       as a "virtual toolhead" to "manual_home". Here, in contrast, the full
        #       toolhead object is passed because it has been modified to support homing
        #       the extruder axis too.
        # NOTE: "PrinterHoming.manual_home" then calls "HomingMove.homing_move".
        logging.info(f"cmd_HOME_EXTRUDER: pos={str(pos)}")
        phoming.manual_home(toolhead=self.toolhead, endstops=endstops,
                            pos=pos, speed=speed,
                            # NOTE: argument passed to "mcu_endstop.home_start",
                            #       and used directly in the low-level command.
                            triggered=True,
                            # NOTE: if True, an "error" is recorded when the move
                            #       completes without the endstop triggering.
                            check_triggered=True)

        # NOTE: Update positions in gcode_move, fixes inaccurate first
        #       relative move. Might not be needed since actually using
        #       set_position from the TH.
        #       Might interfere with extruder move?
        # gcode_move = self.printer.lookup_object('gcode_move')
        # gcode_move.reset_last_position()

        # NOTE: check if the active extruder is the one to be homed.
        if self.extruder_name != self.active_extruder_name:
            try:
                # NOTE: activate the requested extruder if necessary.
                # WARN: using run_script_from_command/run_script instead
                #       would interrupt the homing move and block gcode.
                self.active_extruder.cmd_ACTIVATE_EXTRUDER(gcmd=gcmd)
            except:
                raise gcmd.error("ExtruderHoming.cmd_HOME_EXTRUDER: " +
                                f"Error re-activating {self.active_extruder_name}.")

        # NOTE: flag homing end
        self.homing = False

    cmd_HOME_ACTIVE_EXTRUDER_help = "Home an extruder using an endstop. The active extruder will be homed."
    def cmd_HOME_ACTIVE_EXTRUDER(self, gcmd):

        # NOTE: Get the toolhead and its *current* extruder.
        toolhead: ToolHead = self.printer.lookup_object("toolhead")
        active_extruder = toolhead.get_extruder()           # PrinterExtruder
        # active_extruder_name = active_extruder.get_name()

        # NOTE: Get the active extruder's trapq.
        # extruder_trapq = active_extruder.get_trapq()        # extruder trapq (from ffi)

        # NOTE: Get the steppers
        extruder_stepper = active_extruder.extruder_stepper # ExtruderStepper
        rail = extruder_stepper.rail                        # PrinterRail
        # stepper = extruder_stepper.stepper                  # PrinterRail or PrinterStepper
        # steppers = [stepper]                                # [PrinterRail or PrinterStepper]
        # NOTE: in the "ExtruderStepper" class, the "rail" and the "stepper"
        #       objects are _the same_ object.

        # NOTE: get the endstops from the extruder's PrinterRail.
        #       likely a list of tuples, each with an instance of
        #       MCU_endstop and a stepper name.
        #       See PrinterRail at stepper.py.
        endstops = rail.get_endstops()                      # [(mcu_endstop, name)]

        # NOTE: get a PrinterHoming class from extras
        phoming: PrinterHoming = self.printer.lookup_object('homing')      # PrinterHoming

        # NOTE: Get original toolhead position
        th_orig_pos = toolhead.get_position()

        # NOTE: get homing information, speed and move coordinate.
        homing_info = rail.get_homing_info()
        speed = homing_info.speed
        # NOTE: Use XYZ from the toolhead, and E from the config file + estimation.
        pos = th_orig_pos[:3] + [self.get_movepos(homing_info=homing_info, rail=rail)]

        # Get rail limits
        position_min, position_max = rail.get_range()

        # NOTE: force extruder to a certain starting position.
        #       Originally 0.0, now position_max, which requires an
        #       endstop position of 0.0 to home in the right direction.
        if homing_info.positive_dir:
            # NOTE: pos[-1] is the endstop's position.
            e_startpos = position_min - pos[-1] * 0.1
        else:
            # NOTE: pos[-1] is the endstop's position.
            e_startpos = position_max + pos[-1] * 0.1

        # NOTE: Get the initial position from all non-E elements in the toolhead's
        #       position by using its "axis count" (this can be 3 or 6).
        startpos = th_orig_pos[:-1] + [e_startpos]
        # NOTE: Set the initial position, also permitting limit checks of the extruder axis
        #       to pass (see "homing_axes" argument), which otherwise block homing moves too.
        toolhead.set_position(startpos, homing_axes=tuple([len(startpos)-1]))

        # NOTE: flag homing start
        self.homing = True

        logging.info(f"cmd_HOME_EXTRUDER: pos={str(pos)}")
        phoming.manual_home(toolhead=toolhead, endstops=endstops,
                            pos=pos, speed=speed,
                            # NOTE: argument passed to "mcu_endstop.home_start",
                            #       and used directly in the low-level command.
                            triggered=True,
                            # NOTE: if True, an "error" is recorded when the move
                            #       completes without the endstop triggering.
                            check_triggered=True)

        # NOTE: flag homing end
        self.homing = False

    def get_movepos(self, homing_info, rail=None):
        # NOTE: based on "_home_axis" from CartKinematics, it estimates
        #       the distance to move for homing, at least for a G28 command.

        # NOTE: setup the default rail.
        if rail is None:
            rail = self.rail

        # Determine movement, example config values:
        #   position_endstop: 0.0
        #   position_min: 0.0
        #   position_max: 30.0
        #   homing_positive_dir: False
        # TODO: Review if using "get_range" was important, or remove.
        # position_min, position_max = rail.get_range()

        # NOTE: Use the endstop's position.
        #       The direction of the move towards this point is defined
        #       by the starting position of the move, which is forced
        #       before the homing move. This final position _must_ be
        #       within the limits of the axis being homed.
        movepos = homing_info.position_endstop

        # NOTE: adding a small amount just in case:
        # movepos = 1.1 * movepos  # TODO: check again that this was completely wrong.
        logging.info(f"get_movepos: movepos={str(movepos)}")

        # NOTE: movepos will be the target coordinate for the move,
        #       and will also be the final position registered internally.
        #       This means that GET_POSITION will return an extruder
        #       position equal to movepos (plus trigger point corrections),
        #       for example: E=-33.000625

        return movepos

def load_config_prefix(config):
    return ExtruderHoming(config)
