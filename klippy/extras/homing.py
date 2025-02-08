# Helper code for implementing homing operations
#
# Copyright (C) 2016-2024  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

# Type checking without cyclicc import error.
# See: https://stackoverflow.com/a/39757388
from __future__ import annotations
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from ..toolhead import ToolHead
# pylint: disable=missing-class-docstring,missing-function-docstring,invalid-name,line-too-long,consider-using-f-string,multiple-imports,wrong-import-position
# pylint: disable=logging-fstring-interpolation,logging-not-lazy,fixme

import logging, math
from kinematics.cartesian_abc import CartKinematicsABC

# NOTE:
#   -   https://github.com/Klipper3d/klipper/commit/78f4c25a14099564cf731bdaf5b97492a3a6fb47
#   -   https://github.com/Klipper3d/klipper/commit/dd34768e3afb6b5aa46885109182973d88df10b7
HOMING_START_DELAY = 0.001
ENDSTOP_SAMPLE_TIME = .000015
ENDSTOP_SAMPLE_COUNT = 4

# Return a completion that completes when all completions in a list complete
def multi_complete(printer, completions):
    if len(completions) == 1:
        return completions[0]
    # Build completion that waits for all completions
    reactor = printer.get_reactor()
    cp = reactor.register_callback(lambda e: [c.wait() for c in completions])
    # If any completion indicates an error, then exit main completion early
    for c in completions:
        reactor.register_callback(
            lambda e, c=c: cp.complete(1) if c.wait() else 0)
    return cp

# Tracking of stepper positions during a homing/probing move
class StepperPosition:
    def __init__(self, stepper, endstop_name):
        self.stepper = stepper
        self.endstop_name = endstop_name
        self.stepper_name = stepper.get_name()
        self.start_pos = stepper.get_mcu_position()
        self.start_cmd_pos = stepper.mcu_to_commanded_position(self.start_pos)
        self.halt_pos = self.trig_pos = None
        logging.info(f"homing.StepperPosition: add stepper {self.stepper_name} to endstop {self.endstop_name}")
    def note_home_end(self, trigger_time):
        # NOTE: method called by "homing_move" to determine halt/trig positions.

        # NOTE: uses "itersolve_get_commanded_pos" to read "sk->commanded_pos" (at itersolve.c)
        self.halt_pos = self.stepper.get_mcu_position()
        # NOTE: uses "stepcompress_find_past_position" to:
        #       "Search history of moves to find a past position at a given clock"
        self.trig_pos = self.stepper.get_past_mcu_position(trigger_time)
    def verify_no_probe_skew(self, haltpos):
        new_start_pos = self.stepper.get_mcu_position(self.start_cmd_pos)
        if new_start_pos != self.start_pos:
            logging.warning(
                "Stepper '%s' position skew after probe: pos %d now %d",
                self.stepper.get_name(), self.start_pos, new_start_pos)

# Implementation of homing/probing moves
class HomingMove:
    def __init__(self, printer, endstops, toolhead=None):
        """
        The "HomingMove" class downstream methods use the
        following methods from a provided "toolhead" object:
            - flush_step_generation
            - get_kinematics:           returning a "kin" object with methods:
                - kin.get_steppers:     returning a list of stepper objects.
                - kin.calc_position:    returning ???
            - get_position:             returning "thpos" (toolhead position)
            - get_last_move_time:       returning "print_time" (and later "move_end_print_time")
            - dwell
            - drip_move
            - set_position
        """
        self.printer = printer
        self.endstops = endstops
        if toolhead is None:
            toolhead = printer.lookup_object('toolhead')
        self.toolhead: ToolHead = toolhead
        self.stepper_positions = []

    def get_mcu_endstops(self):
        # NOTE: "self.endstops" is a list of tuples,
        #       containing elements like: (MCU_endstop, "name")
        #       This gets the MCU objects in a simple list.
        return [es for es, name in self.endstops]

    # NOTE: "_calc_endstop_rate" calculates the max amount of steps for the
    #       move, and the time the move will take. It then returns the "rate"
    #       of "time per step".
    def _calc_endstop_rate(self, mcu_endstop, movepos, speed):  # movepos  = [0.0, 0.0, 0.0, -110]
        startpos = self.toolhead.get_position()                 # startpos = [0.0, 0.0, 0.0, 0.0]
        axes_d = [mp - sp for mp, sp in zip(movepos, startpos)]
        move_d = math.sqrt(sum([d*d for d in axes_d[:-1]]))     # 150.0
        move_t = move_d / speed                                 # 150.0 / 25.0 = 6.0
        max_steps = max([(abs(s.calc_position_from_coord(startpos)
                              - s.calc_position_from_coord(movepos))
                          / s.get_step_dist())
                         for s in mcu_endstop.get_steppers()])
        if max_steps <= 0.:
            return .001
        return move_t / max_steps

    def calc_toolhead_pos(self, kin_spos, offsets):
        """Calculate the "actual" halting position in distance units
        The "kin_spos" received here has values from the "halting" position, before "oversteps" are corrected.
        For example:
            calc_toolhead_pos input: kin_spos={'extruder1': 0.0} offsets={'extruder1': -2273}
        
        The "offsets" are probably in "step" units..
        """
        kin_spos = dict(kin_spos)

        # NOTE: log input for reference
        logging.info(f"calc_toolhead_pos input: kin_spos={str(kin_spos)} offsets={str(offsets)}")

        # NOTE: Update XYZ and ABC steppers position.
        for axes in list(self.toolhead.kinematics):
            # Iterate over["XYZ", "ABC"]
            kin: CartKinematicsABC = self.toolhead.kinematics[axes]
            for stepper in kin.get_steppers():
                sname = stepper.get_name()  # NOTE: Example: "stepper_x", "stepper_a", etc.
                # NOTE: update the stepper positions by converting the "offset" steps
                #       to "mm" units and adding them to the original "halting" position.
                kin_spos[sname] += offsets.get(sname, 0) * stepper.get_step_dist()

        # NOTE: Repeat the above for the extruders.
        extruder_steppers = self.printer.lookup_extruder_steppers()  # [ExtruderStepper]
        for extruder_stepper in extruder_steppers:
            for stepper in extruder_stepper.rail.get_steppers():    # PrinterStepper (MCU_stepper)
                sname = stepper.get_name()
                kin_spos[sname] += offsets.get(sname, 0) * stepper.get_step_dist()

        # NOTE: This call to get_position is only used to acquire the extruder
        #       position, and append it to XYZ components below. Example:
        #           thpos=[0.0, 0.0, 0.0, 0.0]
        thpos = self.toolhead.get_position()

        # NOTE: This list is used to define "haltpos", which is then passed to "toolhead.set_position".
        #       It must therefore have enough elements (4 for XYZE, or 7 for XYZABCE).
        result = []

        # NOTE: Run "calc_position" for the XYZ and ABC axes.
        for axes in list(self.toolhead.kinematics):
            # Iterate over["XYZ", "ABC"]
            kin: CartKinematicsABC = self.toolhead.kinematics[axes]
            # NOTE: The "calc_position" method iterates over the rails in the (cartesian)
            #       kinematics and selects "stepper_positions" with matching names.
            #       Perhaps other kinematics do something more elaborate.
            # NOTE: Elements 1-3 from the output are combined with element 4 from "thpos".
            #       This is likely because the 4th element is the extruder, which is not
            #       normally "homeable". So the last position is re-used to form the
            #       updated toolhead position vector.
            # NOTE: Examples (CartKinematics):
            #       -   calc_position input stepper_positions={'extruder': -1.420625}
            #       -   calc_position return pos=[-1.420625, 0.0, 0.0]
            result += list(kin.calc_position(stepper_positions=kin_spos))[:3]

        # TODO: Check if "calc_position" should be run in the extruder kinematics too.
        # NOTE: Ditched "thpos[3:]" (from "toolhead.get_position()" above),
        #       replacing it by the equivalent for the active extruder.
        extruder = self.toolhead.get_extruder()
        if extruder.name is not None:
            result += [kin_spos[extruder.name]]
        else:
            result += [thpos[-1]]

        logging.info(f"calc_toolhead_pos: result={str(result)}")

        # NOTE: This "result" is used to override "haltpos" below, which
        #       is then passed to "toolhead.set_position".
        return result

    def homing_move(self, movepos, speed, probe_pos=False,
                    triggered=True, check_triggered=True):
        """Called by the 'home_rails' or 'manual_home' methods."""
        # Notify start of homing/probing move
        self.printer.send_event(self.toolhead.event_prefix + "homing:homing_move_begin", self)
        logging.info("homing.homing_move: homing move called, starting setup.")

        # Note start location
        self.toolhead.flush_step_generation()

        kin_spos = {}
        # Iterate over["XYZ", "ABC"]
        for axes in list(self.toolhead.kinematics):
            # NOTE: the "get_kinematics" method is defined in the ToolHead
            #       class at "toolhead.py". It apparently returns the kinematics
            #       object, as loaded from a module in the "kinematics/" directory,
            #       during the class's __init__.
            kin: CartKinematicsABC = self.toolhead.kinematics[axes]
            # NOTE: this step calls the "get_steppers" method on the provided
            #       kinematics, which returns a dict of "MCU_stepper" objects,
            #       with names as "stepper_x", "stepper_y", etc.
            kin_spos.update({s.get_name(): s.get_commanded_position()
                             for s in kin.get_steppers()})

        # NOTE: Repeat the above for the extruders, adding them to the "kin_spos" dict.
        #       This is important later on, when calling "calc_toolhead_pos".
        extruder_steppers = self.printer.lookup_extruder_steppers()  # [ExtruderStepper]
        # NOTE: Dummy extruders wont enter the for loop below (as extruder_steppers=[]).
        for extruder_stepper in extruder_steppers:
            # Get PrinterStepper (MCU_stepper) objects.
            for s in extruder_stepper.rail.get_steppers():
                kin_spos.update({s.get_name(): s.get_commanded_position()})

        # NOTE: "Tracking of stepper positions during a homing/probing move".
        #       Build a "StepperPosition" class for each of the steppers
        #       associated to each endstop in the "self.endstops" list of tuples,
        #       containing elements like: (MCU_endstop, "name").
        self.stepper_positions = [ StepperPosition(s, name)
                                   for es, name in self.endstops
                                   for s in es.get_steppers() ]
        # Start endstop checking
        print_time = self.toolhead.get_last_move_time()
        endstop_triggers = []
        logging.info("homing.homing_move: homing move start.")
        for mcu_endstop, name in self.endstops:
            # NOTE: this calls "toolhead.get_position" to get "startpos".
            rest_time = self._calc_endstop_rate(mcu_endstop=mcu_endstop,
                                                movepos=movepos,  # [0.0, 0.0, 0.0, -110.0]
                                                speed=speed)
            # NOTE: "wait" is a "ReactorCompletion" object (from "reactor.py"),
            #       setup by the "home_start" method of "MCU_endstop" (at mcu.py)
            wait = mcu_endstop.home_start(print_time=print_time,
                                          sample_time=ENDSTOP_SAMPLE_TIME,
                                          sample_count=ENDSTOP_SAMPLE_COUNT,
                                          rest_time=rest_time,
                                          triggered=triggered)
            endstop_triggers.append(wait)
        # NOTE: the "endstop_triggers" list contains "reactor.completion" objects.
        #       Those are created by returned by the "home_start" method
        #       of "MCU_endstop" (at mcu.py).
        all_endstop_trigger = multi_complete(printer=self.printer, completions=endstop_triggers)

        # NOTE: This dwell used to be needed by low-power RPi2. Otherwise
        #       calculations would take too long, and by the time they were sent,
        #       the associated "mcu time" would have already passed.
        #       It was not needed after the implementation of drip moves.
        #       I don't know yet why it remains.
        logging.info(f"homing.homing_move: dwell for HOMING_START_DELAY={HOMING_START_DELAY}")
        self.toolhead.dwell(HOMING_START_DELAY)

        # Issue move
        logging.info(f"homing.homing_move: issuing drip move at speed: {speed}")
        error = None
        try:
            # NOTE: Before the "drip" commit, the following command
            #       used to be: self.toolhead.move(movepos, speed)
            #       See: https://github.com/Klipper3d/klipper/commit/43064d197d6fd6bcc55217c5e9298d86bf4ecde7
            self.toolhead.drip_move(newpos=movepos,  # [0.0, 0.0, 0.0, -110.0]
                                    speed=speed,
                                    # NOTE: "all_endstop_trigger" is probably made from
                                    #       the "reactor.completion" objects above.
                                    drip_completion=all_endstop_trigger)
        except self.printer.command_error as e:
            error = "Error during homing move: %s" % (str(e),)

        # Wait for endstops to trigger
        logging.info("homing.homing_move: waiting for endstop triggers.")
        trigger_times = {}
        # NOTE: Probably gets the time just after the last move.
        move_end_print_time = self.toolhead.get_last_move_time()
        for mcu_endstop, name in self.endstops:
            # NOTE: calls the "home_wait" method from "MCU_endstop".
            try:
                trigger_time = mcu_endstop.home_wait(move_end_print_time)
            except self.printer.command_error as e:
                if error is None:
                    error = "Error during homing %s: %s" % (name, str(e))
                continue
            if trigger_time > 0.:
                trigger_times[name] = trigger_time
            elif check_triggered and error is None:
                error = "No trigger on %s after full movement" % (name,)
                # If the trigger time is exactly "0" then the probe was not triggered during the move.

        # Determine stepper halt positions
        # NOTE: "flush_step_generation" calls "flush" on the MoveQueue,
        #       and "_update_move_time" (which updates "self.print_time"
        #       and calls "trapq_finalize_moves").
        self.toolhead.flush_step_generation()

        logging.info("homing.homing_move: calculating haltpos.")
        for sp in self.stepper_positions:
            # NOTE: get the time of endstop triggering
            tt = trigger_times.get(sp.endstop_name, move_end_print_time)
            # NOTE: Record halt position from `stepper.get_mcu_position`,
            #       and trigger position from `stepper.get_past_mcu_position(tt)`
            #       in each StepperPosition class. This information is used below.
            sp.note_home_end(tt)

        # NOTE: calculate halting position from "oversteps" after triggering.
        #       This chunk was added in commit:
        #       https://github.com/Klipper3d/klipper/commit/3814a13251aeca044f6dbbccda706263040e1bec
        if probe_pos:
            # TODO: update G38 to work with ABC axis.
            halt_steps = {sp.stepper_name: sp.halt_pos - sp.start_pos
                          for sp in self.stepper_positions}
            trig_steps = {sp.stepper_name: sp.trig_pos - sp.start_pos
                          for sp in self.stepper_positions}
            haltpos = trigpos = self.calc_toolhead_pos(kin_spos=kin_spos,
                                                       offsets=trig_steps)
            if trig_steps != halt_steps:
                haltpos = self.calc_toolhead_pos(kin_spos=kin_spos,
                                                 offsets=halt_steps)
            self.toolhead.set_position(haltpos)
            for sp in self.stepper_positions:
                sp.verify_no_probe_skew(haltpos)
        else:
            haltpos = trigpos = movepos
            # NOTE: calculate "oversteps" after triggering, for each
            #       StepperPosition class.
            over_steps = {sp.stepper_name: sp.halt_pos - sp.trig_pos
                          for sp in self.stepper_positions}
            if any(over_steps.values()):
                # NOTE: "set_position" calls "flush_step_generation", and
                #       then uses "trapq_set_position" to write the position.
                #       It updates "commanded_pos" on the toolhead, and uses
                #       the "set_position" of the kinematics object (which
                #       uses the set_position method of the rails/steppers).
                #       It ends by emittig a "toolhead:set_position" event.
                self.toolhead.set_position(movepos)

                # NOTE: Get the stepper "halt_kin_spos" (halting positions).
                halt_kin_spos = self.calc_halt_kin_spos(extruder_steppers)

                # NOTE: Calculate the "actual" halting position in distance units.
                haltpos = self.calc_toolhead_pos(kin_spos=halt_kin_spos,
                                                 offsets=over_steps)

            # NOTE: set the toolhead position to the (corrected) halting position.
            # NOTE: for extruder_home this could be:
            #           set_position: input=[-1.420625, 0.0, 0.0, 0.0] homing_axes=()
            #       The fourt element comes from "newpos_e" in the call to
            #       "toolhead.set_position" above. The first element is the corrected
            #       "halt" position.
            logging.info("homing.homing_move: setting position.")
            self.toolhead.set_position(haltpos)

        # Signal homing/probing move complete
        try:
            # NOTE: event received by:
            #       - homing_heaters.py
            #       - probe.py
            #       - tmc.py
            #       Probably not relevant to extruder homing.
            self.printer.send_event(self.toolhead.event_prefix + "homing:homing_move_end", self)
        except self.printer.command_error as e:
            if error is None:
                error = str(e)

        # NOTE: raise any errors if found.
        #       This does not include the "trigger timeout" if
        #       if "check_triggered=False".
        if error is not None:
            raise self.printer.command_error(error)

        # NOTE: returns "trigpos", which is the position of the toolhead
        #       when the endstop triggered.
        logging.info("homing.homing_move: homing move end.")
        return trigpos

    def calc_halt_kin_spos(self, extruder_steppers):
        """Abstraction to calculate halt_kin_spos for all axes on the toolhead (XYZ, ABC, E)."""
        halt_kin_spos = {}
        # Iterate over["XYZ", "ABC"]
        for axes in list(self.toolhead.kinematics):
            kin: CartKinematicsABC = self.toolhead.kinematics[axes]
            # NOTE: Uses "ffi_lib.itersolve_get_commanded_pos",
            #       probably reads the position previously set by
            #       "stepper.set_position" / "itersolve_set_position".
            halt_kin_spos.update({s.get_name(): s.get_commanded_position()
                                 for s in kin.get_steppers()})

        # NOTE: Repeat the above for the extruder steppers (defined above).
        for extruder_stepper in extruder_steppers:
            # Get PrinterStepper (MCU_stepper) objects.
            for s in extruder_stepper.rail.get_steppers():
                halt_kin_spos.update({s.get_name(): s.get_commanded_position()})

        return halt_kin_spos

    def check_no_movement(self, axes=None):
        """
        axes: list of the axes moving in a G38 probing move (x, y, z, extruder/extruder1). See "probe_axes".
        """
        if self.printer.get_start_args().get('debuginput') is not None:
            return None

        logging.info(f"check_no_movement with axes={axes}")

        # NOTE: from the StepperPosition class:
        #       -   self.start_pos = stepper.get_mcu_position()
        #       -   self.trig_pos = self.stepper.get_past_mcu_position(trigger_time)
        for sp in self.stepper_positions:
            sp_name = sp.stepper_name

            if sp.start_pos == sp.trig_pos:
                # NOTE: Optionally return only if the stepper was involved in the probing.

                # NOTE: default behaviour, consider all steppers.
                if axes is None:
                    logging.info(f"check_no_movement matched stepper with default behaviour: {sp_name}")
                    return sp.endstop_name

                # NOTE: This is the "G38" behaviour.
                # NOTE: Handle extruder steppers.
                elif sp_name.lower().startswith("extruder"):
                    if any(axis.lower() == sp_name.lower() for axis in axes):
                        # NOTE: If the stepper that did not move was the "active" one, return.
                        logging.info(f"check_no_movement matched stepper with G38 behaviour: {sp_name}")
                        return sp.endstop_name
                # NOTE: Handle the XYZ axes.
                elif any(axis.lower() in sp_name.lower() for axis in axes if axis in "xyz"):
                    # NOTE: "Will return True if any of the substrings (axis)
                    #       in substring_list (axes) is contained in string (sp_name)."
                    #       See https://stackoverflow.com/a/8122096/11524079
                    logging.info(f"check_no_movement matched stepper with G38 behaviour: {sp_name}")
                    return sp.endstop_name

        return None

# State tracking of homing requests
# NOTE: used here only by the cmd_G28 method from PrinterHoming.
class Homing:
    def __init__(self, printer, toolhead=None):
        # NOTE: Copied over toolhead loading code from "HomingMove".
        if toolhead is None:
            toolhead = printer.lookup_object("toolhead")
        self.toolhead: ToolHead = toolhead
        # NOTE: The normal setup continues.
        self.printer = printer
        self.changed_axes = []
        self.trigger_mcu_pos = {}
        self.adjust_pos = {}
    def set_axes(self, axes):
        self.changed_axes = axes
    def get_axes(self):
        return self.changed_axes
    def get_trigger_position(self, stepper_name):
        return self.trigger_mcu_pos[stepper_name]
    def set_stepper_adjustment(self, stepper_name, adjustment):
        self.adjust_pos[stepper_name] = adjustment
    def _fill_coord(self, coord):
        # Fill in any None entries in 'coord' with current toolhead position
        thcoord = list(self.toolhead.get_position())
        for i in range(len(coord)):
            if coord[i] is not None:
                thcoord[i] = coord[i]
        return thcoord
    def set_homed_position(self, pos):
        self.toolhead.set_position(self._fill_coord(pos))

    def home_rails(self, rails, forcepos, movepos):
        """Called by 'home_axis' at the 'cartesian_abc' kinematics module,
        which calculates the start position (which should be forced) and
        the end position (derived from endstop position parameters).

        Args:
            rails (list): A list of stepper "rail" objects.
            forcepos (list): A list of 4 coordinates, used to force the start position.
            movepos (list): A list of 4 coordinates, used to indicate the target (home) position.
        """
        # NOTE: this method is used by the home method of the
        #       cartesian kinematics, in response to a G28 command.
        # NOTE: The "forcepos" argument is passed 1.5 times the
        #       difference between the endstop position and the
        #       opposing limit coordinate.

        logging.info(f"homing.home_rails: homing begins with forcepos={forcepos} and movepos={movepos}")

        # Notify of upcoming homing operation
        self.printer.send_event(self.toolhead.event_prefix + "homing:home_rails_begin", self, rails)
        # self.printer.send_event("homing:home_rails_begin", self, rails)

        # Alter kinematics class to think printer is at forcepos
        # NOTE: Get the axis IDs of each non-null axis in forcepos.
        homing_axes = [axis for axis in range(self.toolhead.pos_length-1) if forcepos[axis] is not None]
        # NOTE: fill each "None" position values with the
        #       current position (from toolhead.get_position)
        #       of the corresponding axis.
        startpos = self._fill_coord(forcepos)
        homepos = self._fill_coord(movepos)
        # NOTE: esto usa "trapq_set_position" sobre el trapq del XYZ.
        # NOTE: homing_axes se usa finalmente en "CartKinematics.set_position",
        #       para asignarle limites a los "rails" que se homearon.
        self.toolhead.set_position(startpos, homing_axes=homing_axes)

        # Perform first home
        endstops = [es for rail in rails for es in rail.get_endstops()]
        hi = rails[0].get_homing_info()
        hmove = HomingMove(printer=self.printer, endstops=endstops,
                           # NOTE: Force use of a specific toolhead.
                           toolhead=self.toolhead)
        hmove.homing_move(homepos, hi.speed)

        # Perform second home
        if hi.retract_dist:
            # Retract
            startpos = self._fill_coord(forcepos)
            homepos = self._fill_coord(movepos)

            logging.info(f"homing.home_rails: setting up second home startpos={startpos} and homepos={homepos}")

            axes_d = [hp - sp for hp, sp in zip(homepos, startpos)]

            # TODO: consider using all coordinates, not just XYZ(ABC).
            move_d = math.sqrt(sum([d*d for d in axes_d[:-1]]))

            retract_r = min(1., hi.retract_dist / move_d)
            retractpos = [hp - ad * retract_r
                          for hp, ad in zip(homepos, axes_d)]

            logging.info(f"homing.home_rails: issuing retraction move to retractpos={retractpos}")
            self.toolhead.move(retractpos, hi.retract_speed)

            # Home again
            startpos = [rp - ad * retract_r
                        for rp, ad in zip(retractpos, axes_d)]
            self.toolhead.set_position(startpos)
            hmove = HomingMove(self.printer, endstops,
                               # NOTE: Force use of a specific toolhead.
                               toolhead=self.toolhead)
            logging.info(f"homing.home_rails: starting second home startpos={startpos} and homepos={homepos}")
            hmove.homing_move(homepos, hi.second_homing_speed)

            # Check for no movement (endstop deactivation by retraction failed).
            if hmove.check_no_movement() is not None:
                raise self.printer.command_error(
                    "Endstop %s still triggered after retract"
                    % (hmove.check_no_movement(),))
        else:
            logging.info(f"homing.home_rails: homing ended with no second homing move.")

        # Signal home operation complete
        self.toolhead.flush_step_generation()
        self.trigger_mcu_pos = {sp.stepper_name: sp.trig_pos
                                for sp in hmove.stepper_positions}
        self.adjust_pos = {}
        self.printer.send_event(self.toolhead.event_prefix + "homing:home_rails_end", self, rails)
        if any(self.adjust_pos.values()):
            # Apply any homing offsets
            homepos = self.toolhead.get_position()
            kin_spos = {}
            newpos = []
            # Iterate over["XYZ", "ABC"]
            for axes in list(self.toolhead.kinematics):
                # NOTE: the "get_kinematics" method is defined in the ToolHead
                #       class at "toolhead.py". It apparently returns the kinematics
                #       object, as loaded from a module in the "kinematics/" directory,
                #       during the class's __init__.
                kin: CartKinematicsABC = self.toolhead.kinematics[axes]
                # Apply any homing offsets
                # NOTE: this step calls the "get_steppers" method on the provided
                #       kinematics, which returns a dict of "MCU_stepper" objects,
                #       with names as "stepper_x", "stepper_y", etc.
                kin_spos.update({s.get_name(): (s.get_commanded_position() + self.adjust_pos.get(s.get_name(), 0.))
                                for s in kin.get_steppers()})
                # NOTE: Build the "newpos" list with elements from each kinematic.
                newpos.extend(kin.calc_position(kin_spos))

            for axis in homing_axes:
                homepos[axis] = newpos[axis]
            self.toolhead.set_position(homepos)

        logging.info("homing.home_rails: finalized.")

class PrinterHoming:
    def __init__(self, config):
        self.printer = config.get_printer()

        # Main toolhead object "id". It is used instead of loading the toolhead object,
        # because it might not be ready at this stage, but we still need the methods below
        # to be able to grab a different toolhead object when subclassing this elsewhere.
        self.toolhead_id = 'toolhead'

        # Register g-code commands
        gcode = self.printer.lookup_object('gcode')
        gcode.register_command('G28', self.cmd_G28, desc=self.cmd_G28_help)

    def manual_home(self, toolhead, endstops, pos, speed,
                    triggered, check_triggered):
        hmove = HomingMove(self.printer, endstops, toolhead)
        try:
            hmove.homing_move(movepos=pos, speed=speed,
                              # NOTE: # By default "probe_pos" is set to "False".
                              triggered=triggered,
                              check_triggered=check_triggered)
        except self.printer.command_error:
            if self.printer.is_shutdown():
                raise self.printer.command_error(
                    "Homing failed due to printer shutdown")
            raise

    def probing_move(self, mcu_probe, pos, speed, check_triggered=True,
                     # NOTE: Add a "triggered" argument. This is eventually used
                     #       to invert the probing logic at "_home_cmd.send()"
                     #       in "mcu.py" to make the low-level "endstop_home" MCU command.
                     # NOTE: It is passed below to "homing_move". Reusing here the
                     #       default "True" value from that method (to avoid issues
                     #       with other uses of probing_move).
                     triggered=True,
                     # NOTE: "probe_axes" should be a list of the axes
                     #       moving in this probing move.
                     probe_axes=None):
        """
        mcu_probe
        pos
        speed
        check_triggered
        triggered: logic invert for the endstop trigger.
        probe_axes: list of the axes moving in this probing move (x, y, z, extruder/extruder1).
        """

        endstops = [(mcu_probe, "probe")]
        hmove = HomingMove(self.printer, endstops)

        try:
            epos = hmove.homing_move(pos, speed, probe_pos=True,
                                     # NOTE: Pass argument from "probing_move",
                                     #       to support G38.4/5 probing gcodes.
                                     triggered=triggered,
                                     # NOTE: Pass argument from "probing_move",
                                     #       to support G38.3/5 probing gcodes.
                                     check_triggered=check_triggered)
        except self.printer.command_error:
            if self.printer.is_shutdown():
                raise self.printer.command_error(
                    "Probing failed due to printer shutdown")
            raise

        # NOTE: this was getting raised for the G38 moves.
        #       "check_no_movement" looks at the stepper
        #       start and trigger positions. If they are
        #       the same, then the error below is raised.
        #           "Probe triggered prior to movement"
        if hmove.check_no_movement(axes=probe_axes) is not None:
            raise self.printer.command_error(
                "Probe triggered prior to movement")

        # NOTE: "epos" is "trigpos" from the "homing_move" method.
        return epos

    cmd_G28_help = "Run homing procedure"
    def cmd_G28(self, gcmd):
        logging.info(f"PrinterHoming.cmd_G28: homing with command={gcmd.get_commandline()}")

        toolhead = self.printer.lookup_object(self.toolhead_id)
        # Move to origin
        axes = []
        # NOTE: Iterate over XYZ... excluding the E axis.
        for pos, axis in enumerate(list(toolhead.axis_map)[:-1]):
            if gcmd.get(axis, None) is not None:
                if axis not in toolhead.axis_names:
                    raise self.printer.command_error(f"Homing error: you must configure the {axis} axis in order to use it.")
                axes.append(pos)
        if not axes:
            # NOTE: Check if the active extruder can be homed.
            try:
                # NOTE: This will fail if the extruder does not
                #       have a "can_home" attribute.
                home_extruder = toolhead.extruder.can_home
            except:
                home_extruder = False
            # Home the extruder axis if possible.
            if home_extruder:
                axes = toolhead.axis_config
            else:
                # Home all axes, except the extruder axis.
                axes = toolhead.axis_config[:-1]
            # Example axes = [0, 1, 2]
            logging.info(f"PrinterHoming.cmd_G28: no specific axes requested, homing axes={axes}")

        logging.info(f"PrinterHoming.cmd_G28: homing axes={axes}")

        # NOTE: Home all of the requested axes, from their respective kinematics.
        for kin_axes in list(toolhead.kinematics):  # Iterate over ["XYZ", "ABC"].
            # Get kinematics by axis set name (e.g. "XYZ").
            kin = toolhead.kinematics[kin_axes]
            logging.info(f"PrinterHoming.cmd_G28: checking if kin axes={kin.axis} have been requested to home.")
            if any(i in kin.axis for i in axes):
                # NOTE: The "kin.axis" object contains indexes for the axies it handles.
                #       For example: [0, 1, 2] for XYZ, [3, 4] for AB, etc.
                homing_axes = [a for a in axes if a in kin.axis]
                logging.info(f"PrinterHoming.cmd_G28: homing {homing_axes} axes of the {kin.axis} kinematic.")
                self.home_axes(kin=kin, homing_axes=homing_axes)

    def home_axes(self, kin, homing_axes):
        """Home the requested axis on the specified kinematics.

        Args:
            kin (kinematics): Kinematics class for the axes.
            homing_axes (list): List of axis, coherced internally to [0,1,2].

        Raises:
            self.printer.command_error: _description_
        """
        # NOTE: Convert ABC axis IDs to XYZ IDs (i.e. 3,4,5 to 0,1,2).
        #       Not useful, adapting "home_rails" would have been complicated.
        # axes = self.axes_to_xyz(homing_axes)
        axes = homing_axes
        logging.info(f"PrinterHoming.home_axes: homing axis={homing_axes} on toolhead={self.toolhead_id}")

        # NOTE: Instance a "Homing" object, passing it the toolhead of this PrinterHoming instance.
        #       This is important because subclasses of PrinterHoming might be associated to another toolhead.
        toolhead = self.printer.lookup_object(self.toolhead_id)
        homing_state = Homing(printer=self.printer, toolhead=toolhead)

        # NOTE: Update the "self.changed_axes" attribute, to indicate
        #       which axes will be homed (e.g. 0 for X, 1 for Y, ...).
        homing_state.set_axes(axes)

        # NOTE: Let the "kinematics" object decide how to home the requested axes.
        try:
            # NOTE: In the cart kinematics, "kin.home" iterates over each
            #       requested axis, and calls "_home_axis" passing it the axis,
            #       the "homing_state" object (of "Homing" class), and the PrinterRail
            #       object associated to that axis (which has the associated endstop).
            # NOTE: Then "_home_axis" decides which is the starting and end position
            #       of the homing move (startpos=forcepos and endpos=homepos).
            #       It then calls "Homing.home_rails" passing it the positions, which
            #       sets the toolhead position to forcepos, and instantiates a "HomingMove"
            #       object using the endstops from the provided rail, and the "homepos".
            #       "HomingMove.homing_move" is then called to issue the first homing move.
            # NOTE: "HomingMove.homing_move" is responsible for issuing the "toolhead.drip_move"
            #       command (used specifically for homing axes), managing endstop triggers,
            #       doing detailed positon/timing calculations, and other dirtier toolhead stuff.
            kin.home(homing_state)
        except self.printer.command_error:
            if self.printer.is_shutdown():
                raise self.printer.command_error(
                    "Homing failed due to printer shutdown")
            self.printer.lookup_object('stepper_enable').motor_off()
            raise

def load_config(config):
    return PrinterHoming(config)
