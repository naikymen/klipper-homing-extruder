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

import stepper, chelper, logging
from toolhead import Move
from collections import namedtuple
from kinematics.extruder import PrinterExtruder, ExtruderStepper
from extras.homing import PrinterHoming
from gcode import GCodeDispatch
from klippy import Printer
from toolhead import ToolHead

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

        # Not really used by the current move method
        self.HOMING_DELAY = 0.001

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

        logging.info(f"ExtruderHoming: init complete")

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
        self.extruder_stepper = self.extruder.extruder_stepper      # ExtruderStepper
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
        active_extruder_name = active_extruder.get_name()

        # NOTE: Get the active extruder's trapq.
        extruder_trapq = active_extruder.get_trapq()        # extruder trapq (from ffi)

        # NOTE: Get the steppers
        extruder_stepper = active_extruder.extruder_stepper # ExtruderStepper
        rail = extruder_stepper.rail                        # PrinterRail
        stepper = extruder_stepper.stepper                  # PrinterRail or PrinterStepper
        steppers = [stepper]                                # [PrinterRail or PrinterStepper]
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
        toolhead.set_position(startpos, homing_axes=tuple(len(startpos)-1))

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
        position_min, position_max = rail.get_range()

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

    # def get_kinematics(self):
    #     """
    #     Virtual toolhead method.
    #     Called by:
    #         -   HomingMove.calc_toolhead_pos
    #         -   HomingMove.homing_move
    #     """
    #     # TEST: identical to manual_stepper,
    #     #       methods for a "virtual kin" are defined here.
    #     return self

    # def get_steppers(self):
    #     """
    #     Virtual toolhead method.
    #     Called by:
    #         -   HomingMove.calc_toolhead_pos,
    #         -   HomingMove.homing_move

    #     """
    #     # TEST: passes extruder stepper (list)
    #     return self.steppers

    # # TODO: Is this method from manual_stepper required?
    # def sync_print_time(self):
    #     # NOTE: this function is likely making the "toolhead"
    #     #       wait for all moves to end before doing something
    #     #       else. Perhaps it will be called by "manual_home" below.
    #     #toolhead = self.printer.lookup_object('toolhead')
    #     print_time = self.toolhead.get_last_move_time()
    #     if self.next_cmd_time > print_time:
    #         self.toolhead.dwell(self.next_cmd_time - print_time)
    #     else:
    #         self.next_cmd_time = print_time

    # # Toolhead wrappers to support homing
    # def flush_step_generation(self):
    #     """
    #     Virtual toolhead method.
    #     Called by:
    #         -   HomingMove.homing_move
    #     """
    #     # TODO: What should I do here?
    #     #       I think it is allright to use the toolheads method,
    #     #       because the extruder stepper is part of the toolhead,
    #     #       and "trapq_finalize_moves" is eventually called on it too
    #     #       (see "_update_move_time" at "toolhead.py").
    #     #       Also, the alternative "sync_print_time" does not make
    #     #       much sense to me, because it was used to get the latest
    #     #       print time, to be used in other manual_stepper methods.
    #     #self.sync_print_time()
    #     self.toolhead.flush_step_generation()

    # def get_last_move_time(self):
    #     """
    #     Virtual toolhead method.
    #     Called by:
    #         -   HomingMove.homing_move
    #     """
    #     # TODO: What should I do here? Using toolhead method.
    #     # self.sync_print_time()
    #     # return self.next_cmd_time
    #     lmt = self.toolhead.get_last_move_time()
    #     logging.info(f"get_last_move_time: Last move time: {str(lmt)}")
    #     return lmt

    # def dwell(self, delay):
    #     """
    #     Virtual toolhead method.
    #     Called by:
    #         -   HomingMove.homing_move
    #     """
    #     # TODO: What should I do here? Using toolhead method.
    #     #self.next_cmd_time += max(0., delay)
    #     # NOTE: Once upon a time, there was no "drip_move" for homing.
    #     #       so a "dwell" was added to give an old RPi2 enough time
    #     #       to compute stuff:
    #     #       - https://github.com/Klipper3d/klipper/commit/78f4c25a14099564cf731bdaf5b97492a3a6fb47
    #     #       When the drip move was introduced, the dwell was significantly reduced:
    #     #       - https://github.com/Klipper3d/klipper/commit/dd34768e3afb6b5aa46885109182973d88df10b7
    #     #       Here the drip_move is _not_ used, and we thus require the
    #     #       extended dwell time, thereby ignoring the delay argument.

    #     # NOTE: pass the input (default)
    #     self.HOMING_DELAY = delay

    #     # NOTE: From homing.py
    #     # self.HOMING_DELAY = 0.001

    #     # NOTE: The original pre-drip value
    #     # self.HOMING_DELAY = 0.250

    #     # NOTE: the 0.250 valued did not work at all,
    #     #       so it was increased by a nice amount,
    #     #       and now... It works! OMG :D Sometimes...
    #     # self.HOMING_DELAY = 4.0

    #     logging.info(f"dwell: Dwelling for {str(self.HOMING_DELAY)} before homing. Current print_time: {str(self.toolhead.print_time)}")
    #     self.toolhead.dwell(self.HOMING_DELAY)
    #     logging.info(f"dwell: Done sending dwell command. Current print_time: {str(self.toolhead.print_time)}")

    # def move_extruder(self, newpos, speed, drip_completion):
    #     """
    #     This method is an alternative to implement "drip_move",
    #     using the Extruder.move command. To use it instead of other
    #     methods in this class, uncomment it in the "drip_move" method,
    #     and comment the others.
    #     """
    #     # NOTE: Explanation of drip_move arguments:
    #     #       - newpos: new position vector [0,0,0,0]
    #     #       - speed:  ???
    #     #       - drip_completion: ???
    #     # NOTE: some explanation on "drip" moves is available at:
    #     #       https://github.com/Klipper3d/klipper/commit/43064d197d6fd6bcc55217c5e9298d86bf4ecde7
    #     # NOTE: the manual_stepper class simply "moves" the stepper
    #     #       in the regular way. However the ToolHead.drip_move does
    #     #       a lot more, in accordance with the commit linked above.
    #     # self.do_move(newpos[0], speed, self.homing_accel)
    #     # TODO: What should I do here? What should the Move argument be?
    #     #       The "Extruder.move" method requires the following arguments:
    #     #       print_time: ???
    #     #       move: ???
    #     curpos = list(self.toolhead.commanded_pos)
    #     # move = Move(toolhead=self.toolhead,
    #     #             start_pos=curpos,
    #     #             end_pos=curpos[:3] + [25.0],  # TODO: base this on the config
    #     #             speed=self.velocity)
    #     extruder_r = -1.0
    #     newpos_e = 25.0
    #     ntMove = namedtuple('Move', "axes_r accel start_v cruise_v axes_d accel_t cruise_t decel_t start_pos end_pos")
    #     move = ntMove(axes_r=[0,0,0,extruder_r], accel=20.0, start_v=0.0, cruise_v=20.0, axes_d=[None,None,None,newpos_e],
    #                   accel_t=1.0, cruise_t=5.0, decel_t=1.0, start_pos=[0.0,0.0,0.0,0.0], end_pos=[0.0,0.0,0.0,0.0])

    #     # TODO: this should be OK if the dwell above is enough? IDK
    #     print_time = self.toolhead.print_time

    #     self.extruder.move(print_time=print_time, move=move)

    #     # NOTE: the following is done automatically by the end of the move method.
    #     # self.extruder.last_position = 0

    #     # NOTE: estimate next move time and update
    #     next_move_time = (print_time + move.accel_t + move.cruise_t + move.decel_t)
    #     self.toolhead._update_move_time(next_move_time)
    #     self.toolhead.last_kin_move_time = next_move_time

    #     pass

    # def move_toolhead_manual(self, newpos, speed, drip_completion):
    #     """
    #     This method is an alternative to implement "drip_move",
    #     using the ToolHead.manual_move command. To use it instead of other
    #     methods in this class, uncomment it in the "move_toolhead_manual" method,
    #     and comment the others.
    #     """
    #     # NOTE: Explanation of drip_move arguments:
    #     #       - newpos: new position vector [0,0,0,0]
    #     #       - speed:  ???
    #     #       - drip_completion: ???
    #     # NOTE: The manual_move method allows "None" values to be passed,
    #     #       allowing me not to worry about getting the current and new
    #     #       coordinates for the homing move.

    #     # NOTE: using "toolhead.manual_move" allows None values, is simpler
    #     #       to use in that sense, and also ends up calling "toolhead.move".
    #     e_newpos = newpos[3]
    #     coord = [None, None, None, e_newpos]
    #     logging.info(f"move_toolhead: Moving {self.extruder.name} to {str(coord)} for homing.")  # Can be [None, None, None, 0.0]
    #     self.toolhead.manual_move(coord=coord, speed=speed)
    #     logging.info(f"move_toolhead: move completed.")

    #     pass

    # def move_toolhead(self, newpos, speed, drip_completion):
    #     """
    #     This method is an alternative to implement "drip_move",
    #     using the ToolHead.move command. To use it instead of other
    #     methods in this class, uncomment it in the "move_toolhead" method,
    #     and comment the others.
    #     """

    #     # NOTE: using "toolhead.move" should be similar to "toolhead.manual_move".
    #     logging.info(f"move_toolhead: moving toolhead to {str(newpos)} for homing.")
    #     self.toolhead.move(newpos=newpos, speed=speed)
    #     logging.info(f"move_toolhead: move completed.")

    # def move_toolhead_drip(self, newpos, speed, drip_completion):
    #     """
    #     This method passes argument to the real toolhead "drip_move" method.
    #     """

    #     logging.info(f"move_toolhead_drip: drip-moving to {str(newpos)} for homing.")  # Can be [None, None, None, 0.0]
    #     self.toolhead.drip_move(newpos, speed, drip_completion)

    # def move_forced(self, newpos, speed, drip_completion):
    #     """
    #     This method uses MANUAL_MOVE from force_move.py
    #     """
    #     # NOTE: try brute force?
    #     force_move = self.printer.lookup_object("force_move")
    #     pos = -50.0
    #     logging.info(f"move_forced: force-moving a distance of {str(pos)} for homing.")  # Can be [None, None, None, 0.0]
    #     self.stepper = force_move.manual_move(self.stepper, dist=pos, speed=speed, accel=100.0)


    # def drip_move(self, newpos, speed, drip_completion):
    #     """
    #     Virtual toolhead method.
    #     Called by:
    #         -   HomingMove.homing_move
    #     """
    #     # NOTE: the "drip_completion" argument is an object that can be checked
    #     #       for to know if all of the endstops have been triggered yet.

    #     # NOTE: option 1, use the "manual_move" method from the ToolHead class.
    #     # TODO: Seems to work, but it blocks all movement after the home for some time.
    #     #       The problem stems from a "next print time" set too far in the future,
    #     #       probably proportional to the expected time the move takes (~displacement/speed).
    #     #       Can be mitigated with fast homing. Seems to work flawlessly.
    #     # self.move_toolhead_manual(newpos, speed, drip_completion)

    #     # NOTE: option 2, use the "move" method from the Extruder class.
    #     # TODO: Now it seems to work, but it blocks all movement after the home for some time.
    #     #       This is the same issue seen for "move_toolhead_manual".
    #     #self.move_extruder(newpos, speed, drip_completion)

    #     # NOTE: option 3, use the "drop move" method from the ToolHead class.
    #     # TODO: It's strange that the stepper stops at the endstop, and then moves a bit more... it shouldn't!
    #     self.move_toolhead_drip(newpos, speed, drip_completion)

    #     # NOTE: option 4, out of ideas... trying "force move"
    #     # TODO: Now it seems to work, but it blocks all movement after the home for some time.
    #     #       This is the same issue seen for "move_toolhead_manual".
    #     # self.move_forced(newpos, speed, drip_completion)

    #     # NOTE: option 5, use the "move" method from the ToolHead class.
    #     #       This should be almost equal to "move_toolhead_manual".
    #     # TODO: Now it seems to work, but it blocks all movement after the home for some time.
    #     #       This is the same issue seen for "move_toolhead_manual".
    #     # self.move_toolhead(newpos, speed, drip_completion)

    # def get_position(self):
    #     """
    #     Virtual toolhead method.
    #     Called by :
    #         -   _calc_endstop_rate
    #         -   calc_toolhead_pos
    #     """
    #     # NOTE: calc_toolhead_pos only uses the fourth element from this list,
    #     #       which is the extruder position. The equivalent coordinates from
    #     #       the toolhead are obtained from "get_commanded_position" of the
    #     #       steppers in the toolhead kinematics object, which ultimately
    #     #       come from the "ffi" objects.
    #     #       TODO: check that this is the correct method to get the position.
    #     #           Another option is "last_position" from PrinterExtruder, which
    #     #           is updated at the end of a move command (after the trapq_append).
    #     e_pos = self.rail.get_commanded_position()      # NOTE: option 1
    #     # e_pos = self.extruder.last_position           # NOTE: option 2

    #     # NOTE: here I use the first 3 elements of the original toolhead
    #     #       position (which came from toolhead.get_position) and
    #     #       the fourth one comes directly from the extruder rail/stepper.
    #     #       A second option would be to just use "toolhead.get_position",
    #     #       as it returns all of the needed elements
    #     pos = self.th_orig_pos[:3] + [e_pos]  # Option 1
    #     #pos = self.toolhead.get_position()    # Option 2

    #     logging.info(f"get_position output: {str(pos)}")
    #     logging.info(f"get_position current TH pos: {str(self.toolhead.get_position())}")
    #     return pos

    # def set_position(self, newpos, homing_axes=()):
    #     """
    #     Virtual toolhead method.
    #     Called by:
    #         -   HomingMove.homing_move
    #     """

    #     # NOTE: Log stuff
    #     logging.info(f"set_position: input={str(newpos)} homing_axes={str(homing_axes)}")
    #     logging.info(f"set_position: old TH position={str(self.th_orig_pos)}")

    #     # TODO: What should I do here?
    #     # NOTE: I am assuming that the "set_position" applies to steppers.
    #     #       by tracing calls to the "set_position" method in MCU_stepper.
    #     #       There, the "coords" argument is a list of at least 3 components:
    #     #           [coord[0], coord[1], coord[2]]
    #     #       I do not know why it needs three components for a steper object,
    #     #       but from the "itersolve_set_position" code, they seem to be x,y,z
    #     #       components.
    #     # self.do_set_position(newpos[0])
    #     # NOTE: the "toolhead.set_position" method calls "set_position" in a
    #     #       PrinterRail object, which we have here. That method calls
    #     #       the set_position method in each of the steppers in the rail.
    #     # NOTE: At this point, this method receives a vector:
    #     #           "[4.67499999999994, 0.0, 0.0, 3.3249999999999402]"
    #     #                newpos[0]                    newpos[3]
    #     #       The first 3 items come from the "calc_position" method below.
    #     #       The first item is the "updated" position of the extruder stepper,
    #     #       corresponding to "haltpos", and the second and third are hard-coded 0s.
    #     #       The fourth component comes from "rail.get_commanded_position" here,
    #     #       which may correspond to the "old" extruder position, meaning that the
    #     #       rail has not been updated yet (sensibly because set_position is called later).
    #     #       This means that we need the original 123 toolhead components, and
    #     #       only update the remaining extruder component using the toolhead.set_position
    #     #       method. However it only sets the XYZ components in the XYZ "trapq".

    #     # NOTE: This "set_position" is called three times:
    #     #       -   The first (above) receives "startpos": [0.0, 0.0, 0.0, 0.0]  (only the E axis really, the rest is left as it was).
    #     #       -   The second receives "movepos": [0.0, 0.0, 0.0, -110.0]  (just before overstep correction).
    #     #       -   The third receives "haltpos": [-1.420625, 0.0, 0.0, -110.0]  (after overstep correction).
    #     #       The extruder information is, at the first and second times, in the last element
    #     #       of the list, and is equal to "0" or the output from "get_movepos", respectively.
    #     #       However, the third time the E position is in the first list element, and corresponds
    #     #       to the "corrected" extruder position.
    #     #       This cannot be avoided because "calc_toolhead_pos" (a HomingMove method) receives
    #     #       the output from "calc_position" below, only keeping the first three elements
    #     #       (overwritting the fourth element with the fourth element of get_position).

    #     # NOTE: Forcing "0" here means that the corrected "haltpos" is ignored in the "second" call.
    #     # newpos_e = newpos[0]
    #     # newpos_e = newpos[3]
    #     # newpos_e = 0.0
    #     # NOTE: to get the 1st element of the vector after calc_toolhead_pos has run,
    #     #       check if a correction has been calculated below.
    #     if self.corrected_e_pos is None:
    #         # NOTE: use the fourth element by default.
    #         newpos_e = newpos[3]
    #     else:
    #         # NOTE: otherwise, a correction has been made, meaning
    #         #       that we are parsing the output from "calc_toolhead_pos",
    #         #       and the correct "newpos_e" is in the first element.
    #         newpos_e = newpos[0]             # Option 1
    #         #newpos_e = self.corrected_e_pos  # Option 2

    #     # NOTE: setup the final new position vector, using th_orig_pos
    #     #       because it is not expected to change during E homing.
    #     pos = self.th_orig_pos[:3] + [newpos_e]

    #     # NOTE: Log stuff
    #     logging.info(f"set_position: output={str(pos)}")

    #     # TODO: I now need to update the "E" queue somehow.
    #     #       When switching extruder steppers, the Extruder class uses the set_position
    #     #       method in its stepper object, passing it: [extruder.last_position, 0., 0.]
    #     #       Thus, I need a three element list, where the first element is the updated position.

    #     # NOTE: The next line from toolhead.py is: "self.flush_step_generation()"
    #     #       Update extruder position, code adapted from "set_position" in toolhead.py.
    #     # NOTE: The "flush_step_generation" toolhead method runs
    #     #       "trapq_finalize_moves" on the extruder's "trapq" as well.
    #     #       No need to do it here, hopefully.
    #     self.set_position2(newpos_e)

    #     # NOTE: The next line from toolhead.py is: "self.commanded_pos[:] = newpos".
    #     #       The most similar line from extruder.py is in "sync_to_extruder",
    #     #       from the ExtruderStepper class:
    #     #           self.stepper.set_position([extruder.last_position, 0., 0.])
    #     #self.extruder_stepper.stepper.set_position([newpos_e, 0., 0.])
    #     # NOTE: Since the above line is equivalent to the next line , I've commented it.
    #     #       Furthermore, "commanded_pos" *will* be updated in the toolhead by the
    #     #       last line below, calling "toolhead.set_position".

    #     # NOTE: The next line from toolhead.py is: "self.kin.set_position(newpos, homing_axes)"
    #     #       It calls "rail.set_position" on all toolhead's rails (which are PrinterRails),
    #     #       and updates the toolhead limits using "rail.get_range".
    #     #       Those rails are "PrinterRail" classes, which in turn call
    #     #       "stepper.set_position" on each of their steppers.
    #     #       It calls "itersolve_set_position". Replicate here:
    #     self.rail.set_position([newpos_e, 0., 0.])
    #     # NOTE: note that this position will be then read by the
    #     #       call to "stepper.get_commanded_position" in "homing.py".

    #     # NOTE: The next line in toolhead.py is: self.printer.send_event("toolhead:set_position")
    #     #       It runs the "reset_last_position" method in GCodeMove at "gcode_move.py",
    #     #       which takes no arguments. It runs the "position_with_transform" method,
    #     #       which is apparently "toolhead.get_position" (simply returning "commanded_pos").
    #     # NOTE: Thus, I can simply call "set_position" from the toolhead, that does all of the above.
    #     #       This is expected for a complete regular homing move, guessing it shouldn't hurt.
    #     self.toolhead.set_position(pos)

    #     logging.info(f"set_position: final TH position={str(self.toolhead.get_position())}")
    #     pass

    # def set_position2(self, newpos_e):
    #     """Quick and dirty version of set_position."""
    #     # NOTES: See notes from "set_position" above.
    #     pos = self.th_orig_pos[:3] + [newpos_e]
    #     self.toolhead.flush_step_generation()
    #     ffi_main, ffi_lib = chelper.get_ffi()
    #     ffi_lib.trapq_set_position(self.extruder_trapq,
    #                                self.toolhead.print_time,
    #                                newpos_e, 0., 0.)
    #     self.rail.set_position([newpos_e, 0., 0.])
    #     self.toolhead.set_position(pos)

    # def calc_position(self, stepper_positions):
    #     """
    #     Virtual toolhead method.
    #     Called by HomingMove.calc_toolhead_pos
    #     """

    #     logging.info(f"calc_position input stepper_positions={str(stepper_positions)}")

    #     # TODO: What should I do here?
    #     #       The manual_stepper code is similar to the CartKinematics method.
    #     # NOTE: The get_name function is inherited from the
    #     #       first stepper in the steppers list of the
    #     #       PrinterRail class.
    #     # NOTE: This code was grabbed from the cartesian.py "calc_position" method.
    #     self.corrected_e_pos = stepper_positions[self.rail.get_name()]
    #     pos = [self.corrected_e_pos, 0., 0.]

    #     logging.info(f"calc_position return pos={str(pos)}")
    #     return pos

def load_config_prefix(config):
    return ExtruderHoming(config)
