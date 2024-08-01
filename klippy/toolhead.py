# Code for coordinating events on the printer toolhead
#
# Copyright (C) 2016-2024  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

# Type checking without cyclic import error.
# See: https://stackoverflow.com/a/39757388
from __future__ import annotations
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from .klippy import Printer
    from .configfile import ConfigWrapper
    from .gcode import GCodeDispatch
# pylint: disable=missing-class-docstring,missing-function-docstring,invalid-name,line-too-long,consider-using-f-string,multiple-imports,wrong-import-position
# pylint: disable=logging-fstring-interpolation,logging-not-lazy,fixme

import math, logging, importlib
import mcu, chelper, kinematics.extruder
from kinematics.extruder import PrinterExtruder
from pprint import pformat
from collections import namedtuple
# Common suffixes: _d is distance (in mm), _v is velocity (in
#   mm/second), _v2 is velocity squared (mm^2/s^2), _t is time (in
#   seconds), _r is ratio (scalar between 0.0 and 1.0)

""" Notes on 'print_time': https://www.klipper3d.org/Code_Overview.html#time
The print time is synchronized to the main micro-controller clock (the micro-controller defined in the "[mcu]" config section).

It is a floating point number stored as seconds and is relative to when the main mcu was last restarted.

It is possible to convert from a "print time" to the main micro-controller's hardware clock by multiplying the print time by
the mcu's statically configured frequency rate.

The high-level host code uses print times to calculate almost all physical actions (eg, head movement, heater changes, etc.).

Within the host code, print times are generally stored in variables named print_time or move_time.
"""

# Class to track each move request
class Move:
    def __init__(self, toolhead, start_pos, end_pos, speed):
        logging.info(f"Move: setup with start_pos={start_pos} and end_pos={end_pos}")

        self.toolhead = toolhead
        self.start_pos = tuple(start_pos)
        self.end_pos = tuple(end_pos)
        self.accel = toolhead.max_accel
        self.junction_deviation = toolhead.junction_deviation
        self.timing_callbacks = []
        # NOTE: "toolhead.max_velocity" contains the value from the config file.
        #       The "speed" argument comes from the call at "toolhead.move",
        #       which is the feedrate "F" GCODE argument times a factor:
        #           gcode_speed * self.speed_factor
        #       This factor is by default "1. / 60." to convert feedrate units
        #       from mm/min to mm/sec (e.g. F600 is 10 mm/sec).
        velocity = min(speed, toolhead.max_velocity)
        self.is_kinematic_move = True

        # NOTE: amount of non-extruder axes: XYZ=3, XYZABC=6.
        self.axis_names = toolhead.axis_names
        self.axis_count = toolhead.axis_count
        self.limited_axes = toolhead.limited_axes

        # NOTE: Compute the components of the displacement vector.
        #       The last component is now the extruder.
        self.axes_d = axes_d = [ep - sp for ep, sp in zip(end_pos, start_pos)]

        # NOTE: Compute the euclidean magnitude of the XYZ(ABC) displacement vector,
        #       excluding the extruder.
        self.move_d = move_d = math.sqrt(sum([d*d for d in axes_d[:-1]]))

        logging.info(f"Move: setup with axes_d={axes_d} and move_d={move_d}.")

        # NOTE: If the move in XYZ is very small, then parse it as an extrude-only move.
        if move_d < .000000001:
            # Extrude only move

            # NOTE: the main axes wont move, thus end=stop.
            self.end_pos = tuple([sp for sp in start_pos[:-1]])
            # NOTE: the extruder will move.
            self.end_pos = self.end_pos + (end_pos[-1],)

            # NOTE: set axis displacement to zero.
            for i in range(len(axes_d[:-1])):
                axes_d[i] = 0.

            # NOTE: set move distance to the extruder's displacement.
            self.move_d = move_d = abs(axes_d[-1])

            # NOTE: set more stuff (?)
            inv_move_d = 0.
            if move_d:
                inv_move_d = 1. / move_d
            self.accel = 99999999.9
            velocity = speed
            self.is_kinematic_move = False
        else:
            inv_move_d = 1. / move_d

        # NOTE: Compute a ratio between each component of the displacement
        #       vector and the total magnitude. Ratios can be negative.
        self.axes_r = [d * inv_move_d for d in axes_d]

        # NOTE: Scale the acceleration of the move, such that the toolhead's max
        #       acceleration only limits the limited axes.
        self.axes_r_limited = sum([abs(self.axes_r[i]) for i in self.limited_axes])
        if self.axes_r_limited > 0.0:
            self.accel = min(toolhead.max_accel / self.axes_r_limited, 99999999.9)
            logging.info(f"Move: scale acceleration from {toolhead.max_accel} to {self.accel}.")
        else:
            logging.info(f"Move: acceleration set to {self.accel}.")

        # NOTE: Compute the mimimum time that the move will take (at speed == max speed).
        #       The time will be greater if the axes must accelerate during the move.
        self.min_move_t = move_d / velocity

        # Junction speeds are tracked in velocity squared.  The
        # delta_v2 is the maximum amount of this squared-velocity that
        # can change in this move.
        self.max_start_v2 = 0.
        self.max_cruise_v2 = velocity**2
        self.delta_v2 = 2.0 * move_d * self.accel
        self.max_smoothed_v2 = 0.
        self.smooth_delta_v2 = 2.0 * move_d * toolhead.max_accel_to_decel

    def limit_speed(self, speed, accel):
        """Limit the speed of the move, given a maximum velocity and acceleration.
        This method is called from the kinematics, which is in turn caused by calls
        to their 'check_move' methods by 'ToolHead.move'.
        """
        speed2 = speed**2
        if speed2 < self.max_cruise_v2:
            self.max_cruise_v2 = speed2
            self.min_move_t = self.move_d / speed
        self.accel = min(self.accel, accel)
        self.delta_v2 = 2.0 * self.move_d * self.accel
        self.smooth_delta_v2 = min(self.smooth_delta_v2, self.delta_v2)

    def move_error(self, msg="Move out of range"):
        # TODO: check if the extruder axis is always passed to "self.end_pos".
        ep = self.end_pos
        m = msg + ": "
        m += " ".join(["%.3f" % i for i in tuple(ep[:-1])])     # Add XYZABC axis coords.
        m += " [%.3f]" % tuple(ep[-1:])                         # Add extruder coord.
        return self.toolhead.printer.command_error(m)

    def calc_junction(self, prev_move):
        # NOTE: Check if this is an "extruder only" move (i.e. a move not using the
        #       main/non-extruder kinematics) or if the previous one wasn't.
        if not self.is_kinematic_move or not prev_move.is_kinematic_move:
            return

        logging.info(f"Move calc_junction: function triggered. Initial max_start_v2: {self.max_start_v2}")

        # Allow extruder to calculate its maximum junction
        # NOTE: Uses the "instant_corner_v" config parameter.
        extruder_v2 = self.toolhead.extruder.calc_junction(prev_move, self)

        # Find max velocity using "approximated centripetal velocity"
        axes_r = self.axes_r
        prev_axes_r = prev_move.axes_r
        junction_cos_theta = -sum([ axes_r[i] * prev_axes_r[i] for i in range(len(axes_r[:-1])) ])  # NOTE: axes_r comes from axes_d, used is to replace "self.axis_count".
        if junction_cos_theta > 0.999999:
            return
        junction_cos_theta = max(junction_cos_theta, -0.999999)
        sin_theta_d2 = math.sqrt(0.5*(1.0-junction_cos_theta))
        R_jd = sin_theta_d2 / (1. - sin_theta_d2)

        # Approximated circle must contact moves no further away than mid-move
        tan_theta_d2 = sin_theta_d2 / math.sqrt(0.5*(1.0+junction_cos_theta))
        move_centripetal_v2 = .5 * self.move_d * tan_theta_d2 * self.accel
        prev_move_centripetal_v2 = (.5 * prev_move.move_d * tan_theta_d2
                                    * prev_move.accel)
        # Apply limits
        self.max_start_v2 = min(
            R_jd * self.junction_deviation * self.accel,
            R_jd * prev_move.junction_deviation * prev_move.accel,
            move_centripetal_v2, prev_move_centripetal_v2,
            extruder_v2, self.max_cruise_v2, prev_move.max_cruise_v2,
            prev_move.max_start_v2 + prev_move.delta_v2)
        self.max_smoothed_v2 = min(self.max_start_v2,
                                   prev_move.max_smoothed_v2 + prev_move.smooth_delta_v2)

        logging.info(f"Move calc_junction: function end. Final max_start_v2: {self.max_start_v2}")

    def set_junction(self, start_v2, cruise_v2, end_v2):
        """Move.set_junction() implements the "trapezoid generator" on a move.

        The "trapezoid generator" breaks every move into three parts: a constant acceleration phase,
        followed by a constant velocity phase, followed by a constant deceleration phase.
        Every move contains these three phases in this order, but some phases may be of zero duration.

        Args:
            start_v2 (_type_): _description_
            cruise_v2 (_type_): _description_
            end_v2 (_type_): _description_
        """

        logging.info("Move set_junction: function triggered.")

        # Determine accel, cruise, and decel portions of the move distance
        half_inv_accel = .5 / self.accel
        accel_d = (cruise_v2 - start_v2) * half_inv_accel
        decel_d = (cruise_v2 - end_v2) * half_inv_accel
        cruise_d = self.move_d - accel_d - decel_d
        # Determine move velocities
        self.start_v = start_v = math.sqrt(start_v2)
        self.cruise_v = cruise_v = math.sqrt(cruise_v2)
        self.end_v = end_v = math.sqrt(end_v2)
        # Determine time spent in each portion of move (time is the
        # distance divided by average velocity)
        self.accel_t = accel_d / ((start_v + cruise_v) * 0.5)
        self.cruise_t = cruise_d / cruise_v
        self.decel_t = decel_d / ((end_v + cruise_v) * 0.5)

        logging.info("Move set_junction: function end.")

LOOKAHEAD_FLUSH_TIME = 0.250

# Class to track a list of pending move requests and to facilitate
# "look-ahead" across moves to reduce acceleration between moves.
class LookAheadQueue:
    def __init__(self, toolhead):
        self.toolhead = toolhead
        self.queue = []
        self.junction_flush = LOOKAHEAD_FLUSH_TIME
    def reset(self):
        del self.queue[:]
        self.junction_flush = LOOKAHEAD_FLUSH_TIME
    def set_flush_time(self, flush_time):
        self.junction_flush = flush_time
    def get_last(self):
        if self.queue:
            return self.queue[-1]
        return None
    def flush(self, lazy=False):
        """MoveQueue.flush() determines the start and end velocities of each move.

        Args:
            lazy (bool, optional): _description_. Defaults to False.
        """
        # NOTE: logging for tracing activity
        logging.info("MoveQueue flush: function triggered.")
        # NOTE: called by "add_move" when:
        #       "Enough moves have been queued to reach the target flush time."
        #       Also called by "flush_step_generation".

        self.junction_flush = LOOKAHEAD_FLUSH_TIME  # Hardcoded value of "0.250"

        # NOTE: True when "flush" was called by "add_move", in which case
        #       "junction_flush" used to be negative (and was just reset above).
        update_flush_count = lazy

        queue = self.queue

        # NOTE:
        flush_count = len(queue)

        # Traverse queue from last to first move and determine maximum
        # junction speed assuming the robot comes to a complete stop
        # after the last move.
        delayed = []
        next_end_v2 = next_smoothed_v2 = peak_cruise_v2 = 0.
        for i in range(flush_count-1, -1, -1):  # i.e.: "start", "stop", "step".
            move = queue[i]

            # NOTE: "delta_v2" is the maximum amount of this squared-velocity that
            #       can change in this move. "next_end_v2" is initialized to "0" and then
            #       holds "start_v2" of the move that follows (for the remaining iterations).
            # NOTE: Calculate the abosolute maximum (square) speed that can be reached,
            #       by adding the speed change of this move to the start speed of the ¿next?
            reachable_start_v2 = next_end_v2 + move.delta_v2
            # NOTE: "max_start_v2" of the current move is a "speed limit" for the junction
            #       between the moves. Here "start_v2" is set to the minimum between this
            #       maximum juction speed, and the reachable junction speed (makes sense).
            start_v2 = min(move.max_start_v2, reachable_start_v2)

            # NOTE: The math above is now repeated for the
            #       "smoothed versions" of the speeds.
            reachable_smoothed_v2 = next_smoothed_v2 + move.smooth_delta_v2
            smoothed_v2 = min(move.max_smoothed_v2, reachable_smoothed_v2)

            # NOTE: Check if the "max_smoothed_v2" junction speed
            #       was smaller that "reachable_smoothed_v2" just now.
            if smoothed_v2 < reachable_smoothed_v2:
                # It's possible for this move to accelerate
                if (smoothed_v2 + move.smooth_delta_v2 > next_smoothed_v2 or delayed):
                    # This move can either decelerate
                    # and/or is a "full accel" move after a "full decel" move (¿delayed?).
                    if update_flush_count and peak_cruise_v2:
                        # NOTE: The above "update_flush_count" is "True" if the "lazy"
                        #       argument to this function was "True" (as when called
                        #       by "add_move" due to a negative "junction_flush" time).
                        # NOTE: The condition on "peak_cruise_v2" is weird. It will only
                        #       pass when "peak_cruise_v2" is not zero (and this is its
                        #       initialization value). From the comments below it may mean
                        #       that this will trigger "when peak_cruise_v2 is known".
                        flush_count = i
                        update_flush_count = False
                    peak_cruise_v2 = min(move.max_cruise_v2, (smoothed_v2 + reachable_smoothed_v2) * .5)
                    if delayed:
                        # Propagate peak_cruise_v2 to any delayed moves
                        if not update_flush_count and i < flush_count:
                            # NOTE: "i < flush_count" is true by initialization, but may
                            #       be false if "flush_count" was equated to "i" above.
                            mc_v2 = peak_cruise_v2
                            for m, ms_v2, me_v2 in reversed(delayed):
                                mc_v2 = min(mc_v2, ms_v2)
                                m.set_junction(start_v2=min(ms_v2, mc_v2),
                                               cruise_v2=mc_v2,
                                               end_v2=min(me_v2, mc_v2))
                        # NOTE: Moves are removed from the "delayed" list when
                        #       "peak_cruise_v2" is "propagated".
                        del delayed[:]

                # TODO: what is this?
                if not update_flush_count and i < flush_count:
                    # NOTE: "i < flush_count" is true by initialization, but may
                    #       be false if "flush_count" was equated to "i" above.
                    cruise_v2 = min(0.5 * (start_v2 + reachable_start_v2),
                                    move.max_cruise_v2, peak_cruise_v2)
                    move.set_junction(start_v2=min(start_v2, cruise_v2),
                                      cruise_v2=cruise_v2,
                                      end_v2=min(next_end_v2, cruise_v2))
            else:
                # Delay calculating this move until peak_cruise_v2 is known
                delayed.append((move, start_v2, next_end_v2))
            next_end_v2 = start_v2
            next_smoothed_v2 = smoothed_v2

        # NOTE: Here "update_flush_count" is checked to trigger an "early return",
        #       which would skip sending moves to _process_moves (and removing them
        #       from this MoveQueue). "update_flush_count" is True when "lazy=True",
        #       and it remains True if "peak_cruise_v2" is not yet known.
        # NOTE: The other sufficient condition is that the "flush_count" is zero,
        #       which can happen if the queue was originally empty (¿or perhaps if
        #       the peak cruise speed was found on the second move?).
        if update_flush_count or not flush_count:
            logging.info(f"MoveQueue flush: _process_moves skipped due to update_flush_count={update_flush_count} or not flush_count={flush_count}")
            return

        # Generate step times for all moves ready to be flushed
        # NOTE: The clock time when these moves will be executed is not yet explicit,
        #       it will be calculated  by "_process_moves", and then updated with
        #       a call to "_update_move_time".
        logging.info("MoveQueue flush: calling _process_moves.")
        # NOTE: "flush_count" can only have been made possibly smaller by
        #       setting "lazy=True" from the start. This means that a "regular"
        #       call to flush will try to remove all
        self.toolhead._process_moves(moves=queue[:flush_count])

        # Remove processed moves from the queue
        del queue[:flush_count]

    def add_move(self, move):
        """MoveQueue.add_move() places the move object on the "look-ahead" queue.

        Args:
            move (Move): A new Move object.
        """
        logging.info(f"MoveQueue.add_move: adding move.")
        self.queue.append(move)

        # NOTE: The move queue is not flushed automatically when the
        #       new move is the only move in the queue.
        if len(self.queue) == 1:
            return

        # NOTE: "calc_junction" is called on the move, and passed the previous move,
        #       to calculate the values of "max_start_v2" and "max_smoothed_v2" of the
        #       new move.
        move.calc_junction(self.queue[-2])
        # NOTE: "junction_flush" is initialized at 0.250 (see LOOKAHEAD_FLUSH_TIME),
        #       here it is decremented by "min_move_t" of the arriving move. If the
        #       result is less than zero, this signals a "flush" automatically.
        self.junction_flush -= move.min_move_t
        if self.junction_flush <= 0.:
            # Enough moves have been queued to reach the target flush time.
            # NOTE: The "lazy" argument is passed to set "update_flush_count",
            #       to True, which to my surprise, is checked to see if the
            #       "flush_count" variable should be updated (lol).
            self.flush(lazy=True)

BUFFER_TIME_LOW = 1.0
BUFFER_TIME_HIGH = 2.0
BUFFER_TIME_START = 0.250
BGFLUSH_LOW_TIME = 0.200
BGFLUSH_BATCH_TIME = 0.200
BGFLUSH_EXTRA_TIME = 0.250
MIN_KIN_TIME = 0.100

# NOTE: Some insight on this parameter may be available here:
#       https://github.com/Klipper3d/klipper/commit/7ca86f17232e5e0653de512b6322c301b153919c
MOVE_BATCH_TIME = 0.500
STEPCOMPRESS_FLUSH_TIME = 0.050
SDS_CHECK_TIME = 0.001 # step+dir+step filter in stepcompress.c
MOVE_HISTORY_EXPIRE = 30.

DRIP_SEGMENT_TIME = 0.050
DRIP_TIME = 0.100
class DripModeEndSignal(Exception):
    pass

# New trapq class.
class TrapQ:
    def __init__(self):
        ffi_main, ffi_lib = chelper.get_ffi()
        self.trapq = ffi_main.gc(ffi_lib.trapq_alloc(), ffi_lib.trapq_free)
        self.trapq_append = ffi_lib.trapq_append
        self.trapq_finalize_moves = ffi_lib.trapq_finalize_moves
        self.step_generators = []

# Main code to track events (and their timing) on the printer toolhead
class ToolHead:
    """Main toolhead class.

    Example config:

    [printer]
    kinematics: cartesian_abc
    axis: XYZ  # Optional: XYZ or XYZABC
    kinematics_abc: cartesian_abc # Optional
    max_velocity: 5000
    max_z_velocity: 250
    max_accel: 1000

    TODO:
      - The "checks" still have the XYZ logic.
      - Homing is not implemented for ABC.
    """
    def __init__(self, config: ConfigWrapper):
        # NOTE: amount of non-extruder axes: XYZ=3, XYZABC=6.
        self.axis_names = config.get('axis', 'XYZ')  # e.g. "XYZ", "XYZABC", "XY".
        self.axis_count = len(self.axis_names)

        # Axis sets and names for them are partially hardcoded all around.
        self.axis_triplets = ["XYZ", "ABC"]  # TODO: generalize the code to support "UVW" axes.
        self.ax_letters = "".join(self.axis_triplets)
        for l in self.axis_names:
            if l not in self.ax_letters:
                msg = f"ToolHead config error: axis '{l}' is not allowed. Allowed values are '{self.ax_letters}'."
                logging.exception(msg)
                raise config.error(msg)
        # Find the minimum amount of axes needed for the requested axis triplets.
        # For example, 1 triplet would be required for "XYZ" or "ABC", but 2
        # triplets are needed for any mixing of those (e.g. "XYZAB").
        self.min_axes = 3 * sum([ 1 for axset in self.axis_triplets if set(self.axis_names).intersection(axset) ])

        # Length for the position vector, matching the required axis,
        # plus 1 for the extruder axis (even if it is a dummy one).
        self.pos_length = self.min_axes + 1
        # self.pos_length = self.axis_count + 1
        # NOTE: The value of this attriute must match the one at "gcode_move.py".

        # Dictionary to map axes to their indexes in the position vector "self.commanded_pos".
        self.axis_map = {a: i for i, a in enumerate(list(self.ax_letters)[:self.min_axes] + ["E"])}
        # Full set of configured axes indexes, including the extruder.
        self.axis_config = [self.axis_map[x] for x in (self.axis_names + "E")]
        msg = f"ToolHead: setup axis_map to '{self.axis_map}' and axis_config to '{self.axis_config}'."
        logging.info(msg)

        # Which of the kinematic (non-extruder) axes are limited by the general acceleration setting.
        self.accel_limited_axes = config.get('accel_limited_axes', self.axis_names)  # e.g. "XYZ", "XYZABC", "XY".
        # Check.
        if not all([n in self.axis_names for n in self.accel_limited_axes]):
            msg = f"ToolHead setup error: all accel limited axes ({self.accel_limited_axes})"
            msg += f" must be in the configured axis names ({self.axis_names})."
            logging.exception(msg)
            raise config.error(msg)
        # Same thing, but in integer representation.
        self.limited_axes = [self.axis_map[n] for n in self.accel_limited_axes]

        # TODO: support more kinematics.
        self.supported_kinematics = ["cartesian_abc", "corexy_abc", "none"]  # Removed "cartesian" until I fix it.

        msg = f"ToolHead: starting setup with axes={self.axis_names}, pos_length={self.pos_length}"
        msg += f", and accel_limited_axes={self.accel_limited_axes}"
        logging.info(msg)

        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.all_mcus = [
            m for n, m in self.printer.lookup_objects(module='mcu')]
        self.mcu = self.all_mcus[0]
        self.lookahead = LookAheadQueue(self)
        self.lookahead.set_flush_time(BUFFER_TIME_HIGH)

        # Initiate position as a zero vector.
        self.commanded_pos = [0.0 for i in range(self.pos_length)]

        # Toolhead object name/ID
        self.name = "toolhead"
        self.extra_toolheads = {}

        # Prefix for event names
        self.event_prefix = ""

        # Velocity and acceleration control
        self.max_velocity = config.getfloat('max_velocity', above=0.)
        self.max_accel = config.getfloat('max_accel', above=0.)
        min_cruise_ratio = 0.5
        if config.getfloat('minimum_cruise_ratio', None) is None:
            req_accel_to_decel = config.getfloat('max_accel_to_decel', None,
                                                 above=0.)
            if req_accel_to_decel is not None:
                config.deprecate('max_accel_to_decel')
                min_cruise_ratio = 1. - min(1., (req_accel_to_decel
                                                 / self.max_accel))
        self.min_cruise_ratio = config.getfloat('minimum_cruise_ratio',
                                                min_cruise_ratio,
                                                below=1., minval=0.)
        self.square_corner_velocity = config.getfloat(
            'square_corner_velocity', 5., minval=0.)
        self.junction_deviation = self.max_accel_to_decel = 0.
        self._calc_junction_deviation()

        # Input stall detection
        self.check_stall_time = 0.
        self.print_stall = 0
        # Input pause tracking
        self.can_pause = True
        if self.mcu.is_fileoutput():
            self.can_pause = False
        self.need_check_pause = -1.
        # Print time tracking
        self.print_time = 0.
        self.special_queuing_state = "NeedPrime"
        self.priming_timer = None
        self.drip_completion = None
        # Flush tracking
        self.flush_timer = self.reactor.register_timer(self._flush_handler)
        self.do_kick_flush_timer = True
        self.last_flush_time = self.min_restart_time = 0.
        self.need_flush_time = self.step_gen_time = self.clear_history_time = 0.
        # Kinematic step generation scan window time tracking
        self.kin_flush_delay = SDS_CHECK_TIME
        self.kin_flush_times = []
        # Setup iterative solver
        ffi_main, ffi_lib = chelper.get_ffi()
        self.trapq_append = ffi_lib.trapq_append
        self.trapq_finalize_moves = ffi_lib.trapq_finalize_moves
        self.step_generators = []

        # NOTE: check TRAPQ for the extra ABC axes here.
        # TODO: rewite this part to setup an arbitrary amount of axis, relying on the specification (XYZABC).
        #if len(self.axis_names) == 0:
        #    msg = f"Error loading toolhead with '{self.axis_names}' ({len(self.axis_names)})."
        #    msg += " At least one axis must be configured. Use 'none' kinematics otherwise."
        #    logging.exception(msg)
        #    raise config.error(msg)
        if len(self.axis_names) > 6:
            msg = f"Error loading toolhead with '{self.axis_names}' ({len(self.axis_names)})."
            msg += " No more than 6 axes can be configured, yet. Write an issue if you want this."
            logging.exception(msg)
            raise config.error(msg)
        elif self.min_axes > 3:
            logging.info("ToolHead: setting up additional ABC trapq.")

        # NOTE: load the gcode objects (?)
        gcode: GCodeDispatch = self.printer.lookup_object('gcode')
        self.Coord = gcode.Coord

        # NOTE: Load trapq (iterative solvers) and kinematics for the requested axes.
        self.kinematics = {}
        self.load_axes(config=config)

        # Create extruder kinematics class
        # NOTE: setup a dummy extruder at first, replaced later if configured.
        self.extruder = kinematics.extruder.DummyExtruder(self.printer)

        # Register g-code commands
        handlers = [
            'G4', 'M400', 'M204', 'SET_VELOCITY_LIMIT'
        ]

        # NOTE: this iterates over the commands above and finds the functions
        #       and description strings by their names (as they appear in "handlers").
        for cmd in handlers:
            func = getattr(self, 'cmd_' + cmd)
            desc = getattr(self, 'cmd_' + cmd + '_help', None)
            gcode.register_command(cmd, func, when_not_ready=False, desc=desc)

        gcode.register_command('GET_STATUS_MSG', self.get_status_msg,
                               desc=self.cmd_GET_STATUS_MSG_help)

        self.printer.register_event_handler("klippy:shutdown",
                                            self._handle_shutdown)
        # Load some default modules
        modules = ["gcode_move", "homing", "idle_timeout", "statistics",
                   "manual_probe", "tuning_tower"]
        for module_name in modules:
            self.printer.load_object(config, module_name)

    # Load axes abstraction
    def load_axes(self, config):
        """Convnenience function to setup kinematics and trapq objects for the toolhead.

        The definition of this function contains several "hardcoded" variables that should
        be moved to a separate config file eventually, or be otherwise configurable.

        Args:
            config (_type_): Klipper configuration object.
        """
        # Get and setup XYZ axes.
        xyz_axes = ''.join([ax for ax in self.axis_names if ax in "XYZ"])       # e.g. "XY"
        xyz_ids = [i for i, ax in enumerate(self.axis_names) if ax in "XYZ"]    # e.g. "[0, 1]"
        if xyz_axes:
            # Create XYZ kinematics class, and its XYZ trapq (iterative solver).
            self.kin, self.trapq = self.setup_kinematics(config=config,
                                                         config_name='kinematics',
                                                         axes_ids=xyz_ids,
                                                         axis_set_letters=xyz_axes)
            # Save the kinematics to the dict.
            self.kinematics["XYZ"] = self.kin
        else:
            self.kin, self.trapq = None, None

        # Setup ABC axes
        abc_axes = ''.join([ax for ax in self.axis_names if ax in "ABC"])       # e.g. "AB"
        abc_ids = [i for i, ax in enumerate(self.axis_names) if ax in "ABC"]    # e.g. "[3, 4]"
        if abc_axes:
            # Create ABC kinematics class, and its ABC trapq (iterative solver).
            self.kin_abc, self.abc_trapq = self.setup_kinematics(config=config,
                                                                 config_name='kinematics_abc',
                                                                 axes_ids=abc_ids, # e.g. [3, 4 ,5]
                                                                 axis_set_letters=abc_axes)
            # Save the kinematics to the dict.
            self.kinematics["ABC"] = self.kin_abc
        else:
            self.kin_abc, self.abc_trapq = None, None

        # Save the position indexes for the selected axes.
        self.axes = xyz_ids + abc_ids
        # Add the extruder axis.
        self.axes += [self.axis_map["E"]]

    # Load kinematics object
    def setup_kinematics(self, config, axes_ids, config_name='kinematics', axis_set_letters="XYZ"):
        """Load kinematics for a set of axes.

        Note: this requires the Kinematics module to accept a "trapq" object,
        which it must use. Thus, the "load_kinematics" function must also
        be able to pass the object to the instantiation of the new knimeatics class.

        Most kinematics in this branch have not been updated.

        Args:
            config (_type_): Klipper configuration object.
            axes_ids (list): List of integers specifying which of the "toolhead position" elements correspond to the axes of the new kinematic. Examples: [0, 1, 2] for XYZ, or [3, 4 ,5] for ABC.
            config_name (str, optional): Name of the kinematics setting in the config. The configured value must be one of the supported kinematics (e.g. "cartesian_abc"). Defaults to 'kinematics'.
            axis_set_letters (str, optional): Letters identifying each of the three axes in the set. Defaults to 'XYZ'.

        Returns:
            CartKinematics: Kinematics object.
        """
        # NOTE: Get the "kinematics" type from the "[printer]" config section.
        # NOTE: No default value is passed, forcing the user to make a choice.
        kin_name = config.get(config_name)

        # # Handle the "none" kinematics as a special case.
        # if kin_name == "none":
        #     trapq = None
        #     kin = none.load_kinematics(toolhead=self, config=config, trapq=trapq)
        #     return kin, trapq

        # TODO: Support other kinematics is due. Error out for now.
        if kin_name not in self.supported_kinematics:
            msg = f"Error loading kinematics '{kin_name}'. Currently supported kinematics: {self.supported_kinematics}"
            logging.exception(msg)
            raise config.error(msg)
        else:
            logging.info(f"Loading kinematics {kin_name}.")

        # Create a Trapq for the kinematics
        ffi_main, ffi_lib = chelper.get_ffi()
        trapq = ffi_main.gc(ffi_lib.trapq_alloc(), ffi_lib.trapq_free)  # TrapQ()

        # Set up the kinematics object
        try:
            # Import the python module file for the requested kinematic.
            mod = importlib.import_module('kinematics.' + kin_name)
            # Run the modules setup function.
            kin = mod.load_kinematics(toolhead=self, config=config, trapq=trapq,
                                      # Specify which of the "toolhead position" elements correspond to the new set of axes.
                                      axes_ids=axes_ids.copy(),             # e.g. [1,2]
                                      axis_set_letters=axis_set_letters)    # e.g. "XY"
        except config.error as e:
            raise
        except self.printer.lookup_object('pins').error as e:
            raise
        except:
            msg = "Error loading kinematics '%s'" % (kin_name,)
            logging.exception(msg)
            raise config.error(msg)

        return kin, trapq

    # Print time and flush tracking
    def _advance_flush_time(self, flush_time):
        """Flush steps from itersolve and update "print_time".
        It should have a better name, because it is the function that actually sends the steps. Previously named "_update_move_time".
        """
        # NOTE: This function updates "self.print_time" directly, periodically
        #       flushing moves until it is greater than the requested "flush_time".
        #       It updates "self.print_time" until it is greater than
        #       the provided "flush_time", flushing moves in the
        #       "itersolve" queue in small time chunks (probably to
        #       "Generate steps for moves" )
        # NOTE: It also calls "trapq_finalize_moves" on the extruder and toolhead,
        #       "flush_moves" on all MCUs, and "generate_steps" on all steppers.
        # NOTE: Called by "flush_step_generation", "_process_moves",
        #       "dwell", and "_update_drip_move_time".
        logging.info(f"ToolHead: _update_move_time triggered with flush_time={flush_time}")
        flush_time = max(flush_time, self.last_flush_time)
        # Generate steps via itersolve
        sg_flush_want = min(flush_time + STEPCOMPRESS_FLUSH_TIME,
                            self.print_time - self.kin_flush_delay)
        sg_flush_time = max(sg_flush_want, flush_time)
        for sg in self.step_generators:
            # NOTE: "self.step_generators" has been populated with "generate_steps" functions,
            #       one per stepper, by each kinematic class (including the extruder class).
            #       Those functions in turn end up calling "ffi_lib.itersolve_generate_steps"
            #       which are meant to "Generate step times for a range of moves on the trapq".
            sg(sg_flush_time)
        self.min_restart_time = max(self.min_restart_time, sg_flush_time)
        # Free trapq entries that are no longer needed
        clear_history_time = self.clear_history_time
        if not self.can_pause:
            clear_history_time = flush_time - MOVE_HISTORY_EXPIRE
        # NOTE: Expire moves in the trapq before the "free_time" time.
        #       This is defined as "self.force_flush_time", unless it is
        #       less than "print_time-kin_flush_delay*2" (equivalent in the
        #       case that "fft < self.print_time - kin_flush_delay" avobe).
        free_time = sg_flush_time - self.kin_flush_delay
        # NOTE: "free_time" is smaller than "sg_flush_time" by "kin_flush_delay",
        #       which is defined from "SDS_CHECK_TIME".
        # TODO: remove old (pre 6-axis) stuff.
        # # NOTE: Update move times on the toolhead, meaning:
        # #           "Expire any moves older than `free_time` from
        # #           the trapezoid velocity queue" (see trapq.c).
        # self.trapq_finalize_moves(self.trapq, free_time)
        # # NOTE: Setup "self.trapq_finalize_moves" on the ABC trapq as well.
        # self.trapq_finalize_moves(self.abc_trapq, free_time)
        # NOTE: Update move times on the extruder by calling
        #       "trapq_finalize_moves" in PrinterExtruder.
        # NOTE: Update move times on the toolhead's trapqs, meaning:
        #       "Expire any moves older than `free_time` from
        #       the trapezoid velocity queue" (see trapq.c).
        for axes in list(self.kinematics):
            # Iterate over ["XYZ", "ABC"].
            kin = self.kinematics[axes]
            logging.info(f"ToolHead._update_move_time calling trapq_finalize_moves on axes={axes} with free_time={free_time}")
            self.trapq_finalize_moves(kin.trapq, free_time, clear_history_time)
        self.extruder.update_move_time(free_time, clear_history_time)
        # Flush stepcompress and mcu steppersync
        for m in self.all_mcus:
            # NOTE: The following may find and transmit any scheduled steps
            #       prior to the given 'mcu_flush_time' (see stepcompress.c
            #       and "flush_moves" in mcu.py).
            m.flush_moves(flush_time, clear_history_time)
        self.last_flush_time = flush_time

    def _advance_move_time(self, next_print_time):
        pt_delay = self.kin_flush_delay + STEPCOMPRESS_FLUSH_TIME
        flush_time = max(self.last_flush_time, self.print_time - pt_delay)
        self.print_time = max(self.print_time, next_print_time)
        want_flush_time = max(flush_time, self.print_time - pt_delay)
        while 1:
            flush_time = min(flush_time + MOVE_BATCH_TIME, want_flush_time)
            self._advance_flush_time(flush_time)
            if flush_time >= want_flush_time:
                # NOTE: The loop breaks when the update print_time is
                #       greater than the requested update time.
                break

    def _calc_print_time(self):
        # NOTE: Called during "special" queuing states,
        #       by "get_last_move_time" or "_process_moves".
        # NOTE: This function updates "self.print_time" directly.
        # NOTE: Also sends a "toolhead:sync_print_time" event, handled by
        #       "handle_sync_print_time" at "idle_timeout.py". It calls
        #       "reactor.update_timer" and sends an "idle_timeout:printing"
        #       event (which is only handled by tmc2660.py).

        # NOTE: Get the current (host) system time.
        curtime = self.reactor.monotonic()

        # NOTE: Method from MCU (at mcu.py) that calls the
        #       "self._clocksync.estimated_print_time"
        #       method from the ClockSync class (at clocksync.py).
        #       The method uses "get_clock" to get "self.clock_est"
        #       from the ClockSync class. That object is updated in
        #       the background by "_handle_clock" which:
        #       "is invoked from background thread" for "MCU clock querying".
        est_print_time = self.mcu.estimated_print_time(curtime)

        # NOTE: Guessing that the following adds potential delays to
        #       the MCU time, estimating a "minimum print time".
        kin_time = max(est_print_time + MIN_KIN_TIME, self.min_restart_time)
        kin_time += self.kin_flush_delay
        min_print_time = max(est_print_time + BUFFER_TIME_START, kin_time)

        if min_print_time > self.print_time:
            self.print_time = min_print_time
            self.printer.send_event(self.event_prefix + "toolhead:sync_print_time",  # "toolhead:sync_print_time"
                                    curtime, est_print_time, self.print_time)
    def _process_moves(self, moves):
        """
        When ToolHead._process_moves() is called, everything about the move is known - its start location,
        its end location, its acceleration, its start/cruising/end velocity, and distance traveled during
        acceleration/cruising/deceleration.
        All the information is stored in the Move() class and is in cartesian space in units of millimeters and seconds.

        Klipper uses an iterative solver to generate the step times for each stepper. For efficiency reasons,
        the stepper pulse times are generated in C code. The moves are first placed on a "trapezoid motion queue":
            ToolHead._process_moves() -> trapq_append() (in klippy/chelper/trapq.c).

        Note that the extruder is handled in its own kinematic class:
            ToolHead._process_moves() -> PrinterExtruder.move()
        Since the Move() class specifies the exact movement time and since step pulses are sent to the micro-controller
        with specific timing, stepper movements produced by the extruder class will be in sync with head movement even
        though the code is kept separate.

        The step times are then generated:
            ToolHead._process_moves() -> ToolHead._update_move_time() -> MCU_Stepper.generate_steps() ->
            itersolve_generate_steps() -> itersolve_gen_steps_range() (in klippy/chelper/itersolve.c).

        The goal of the iterative solver is to find step times given a function that calculates a stepper
        position from a time. This is done by repeatedly "guessing" various times until the stepper position
        formula returns the desired position of the next step on the stepper. The feedback produced from each
        guess is used to improve future guesses so that the process rapidly converges to the desired time.

        The kinematic stepper position formulas are located in the klippy/chelper/ directory (eg, kin_cart.c,
        kin_corexy.c, kin_delta.c, kin_extruder.c).

        After the iterative solver calculates the step times they are added to an array:
            itersolve_gen_steps_range() -> stepcompress_append() (in klippy/chelper/stepcompress.c).

        The next major step is to compress the steps:
            stepcompress_flush() -> compress_bisect_add() (in klippy/chelper/stepcompress.c)

        Args:
            moves (_type_): _description_
        """
        # NOTE: this ToolHead method is called during the execution of
        #       the "flush" method in a "MoveQueue" class instance.
        #       The "moves" argument receives a "queue" of moves "ready to be flushed".

        # NOTE: logging for tracing activity
        logging.info("ToolHead _process_moves: function triggered.")

        # Resync print_time if necessary
        if self.special_queuing_state:
            if self.special_queuing_state != "Drip":
                # Transition from "NeedPrime"/"Priming" state to main state
                self.special_queuing_state = ""
                self.need_check_pause = -1.
            # NOTE Update "self.print_time".
            self._calc_print_time()
            # NOTE: Also sends a "toolhead:sync_print_time" event.
            logging.info(f"ToolHead _process_moves: self.print_time={str(self.print_time)}")

        # Queue moves into trapezoid motion queue (trapq)
        # NOTE: the "trapq" is possibly something like a CFFI object.
        #       From the following I interpret that it is actually this
        #       object the one responsible for sending commands to
        #       the MCUs.
        next_move_time = self.print_time
        for move in moves:
            logging.info(f"ToolHead _process_moves: next_move_time={str(next_move_time)}")

            for axes in list(self.kinematics):
                # Iterate over["XYZ", "ABC"]
                logging.info(f"toolhead._process_moves: appending move to {axes} trapq.")
                kin = self.kinematics[axes]
                # NOTE: The moves are first placed on a "trapezoid motion queue" with trapq_append.
                if move.is_kinematic_move:
                    self.trapq_append(
                        kin.trapq, next_move_time,
                        move.accel_t, move.cruise_t, move.decel_t,
                        # NOTE: "kin.axis" is used to select the position value that corresponds
                        #       to the current kinematic axis (e.g. kin.axis is [0,1,2] for the XYZ axis,
                        #       or [3,4,5] for the ABC axis).
                        move.start_pos[kin.axis[0]], move.start_pos[kin.axis[1]], move.start_pos[kin.axis[2]],
                        move.axes_r[kin.axis[0]], move.axes_r[kin.axis[1]], move.axes_r[kin.axis[2]],
                        move.start_v, move.cruise_v, move.accel)

            # NOTE: Repeat for the extruder's trapq.
            if move.axes_d[-1]:
                # NOTE: The extruder stepper move is likely synced to the main
                #       XYZ movement here, by sharing the "next_move_time"
                #       parameter in the call.
                self.extruder.move(print_time=next_move_time, move=move)

            # NOTE: The start MCU time for the next move in
            #       the move queue is calculated here.
            next_move_time = (next_move_time + move.accel_t
                              + move.cruise_t + move.decel_t)

            # NOTE: Execute any "callbacks" registered
            #       to be run at the end of this move.
            for cb in move.timing_callbacks:
                cb(next_move_time)

        # Generate steps for moves
        if self.special_queuing_state:
            # NOTE: this block is executed when "special_queuing_state" is not None.
            # NOTE: loging "next_move_time" for tracing.
            logging.info("ToolHead _process_moves: calling _update_drip_move_time with " +
                         f"next_move_time={str(next_move_time)}")
            # NOTE: This function loops "while self.print_time < next_print_time".
            #       It "pauses before sending more steps" using "drip_completion.wait",
            #       and calls "_update_move_time" with small increments in "next_move_time".
            #       If the "DripModeEndSignal" is raised, the code below is skipped, and
            #       returns to "drip_move".
            self._update_drip_move_time(next_move_time)
        self.note_mcu_movequeue_activity(next_move_time + self.kin_flush_delay,
                                         set_step_gen_time=True)
        self._advance_move_time(next_move_time)
    def _flush_lookahead(self):
        # Transit from "NeedPrime"/"Priming"/"Drip"/main state to "NeedPrime"
        # NOTE: This is the "flush" method from a "LookAhead" object.
        #       Llikely sending all moves and removing them from the "move_queue".
        self.lookahead.flush()
        self.special_queuing_state = "NeedPrime"
        self.need_check_pause = -1.
        self.lookahead.set_flush_time(BUFFER_TIME_HIGH)
        self.check_stall_time = 0.
    def flush_step_generation(self):
        self._flush_lookahead()
        self._advance_flush_time(self.step_gen_time)
        self.min_restart_time = max(self.min_restart_time, self.print_time)
    def get_last_move_time(self):
        # NOTE: this method probably returns a "safe" time
        #       which can be used to schedule a new move,
        #       after others have finished.
        # NOTE: If not in a "special_queuing_state" it will only
        #       call "move_queue.flush" and return "self.print_time".
        #       This would cause a print_time update if any moves in
        #       the queue are casued to be sent to "_process_moves".
        #       On a special state it will also flush the "itersolve queue".
        if self.special_queuing_state:
            self._flush_lookahead()
            # NOTE: the "_calc_print_time" function also updates "self.print_time"
            self._calc_print_time()
        else:
            self.lookahead.flush()
        return self.print_time

    def _check_pause(self):
        eventtime = self.reactor.monotonic()
        est_print_time = self.mcu.estimated_print_time(eventtime)
        buffer_time = self.print_time - est_print_time
        if self.special_queuing_state:
            if self.check_stall_time:
                # Was in "NeedPrime" state and got there from idle input
                if est_print_time < self.check_stall_time:
                    self.print_stall += 1
                self.check_stall_time = 0.
            # Transition from "NeedPrime"/"Priming" state to "Priming" state
            self.special_queuing_state = "Priming"
            self.need_check_pause = -1.
            if self.priming_timer is None:
                self.priming_timer = self.reactor.register_timer(
                    self._priming_handler)
            wtime = eventtime + max(0.100, buffer_time - BUFFER_TIME_LOW)
            self.reactor.update_timer(self.priming_timer, wtime)
        # Check if there are lots of queued moves and pause if so
        while 1:
            pause_time = buffer_time - BUFFER_TIME_HIGH
            if pause_time <= 0.:
                break
            if not self.can_pause:
                self.need_check_pause = self.reactor.NEVER
                return
            eventtime = self.reactor.pause(eventtime + min(1., pause_time))
            est_print_time = self.mcu.estimated_print_time(eventtime)
            buffer_time = self.print_time - est_print_time
        if not self.special_queuing_state:
            # In main state - defer pause checking until needed
            self.need_check_pause = est_print_time + BUFFER_TIME_HIGH + 0.100
    def _priming_handler(self, eventtime):
        self.reactor.unregister_timer(self.priming_timer)
        self.priming_timer = None
        try:
            if self.special_queuing_state == "Priming":
                self._flush_lookahead()
                self.check_stall_time = self.print_time
        except:
            logging.exception("Exception in priming_handler")
            self.printer.invoke_shutdown("Exception in priming_handler")
        return self.reactor.NEVER
    def _flush_handler(self, eventtime):
        """Callback function for the 'self.flush_timer' reactor timer"""
        try:
            est_print_time = self.mcu.estimated_print_time(eventtime)
            if not self.special_queuing_state:
                # In "main" state - flush lookahead if buffer runs low
                print_time = self.print_time
                buffer_time = print_time - est_print_time
                if buffer_time > BUFFER_TIME_LOW:
                    # Running normally - reschedule check
                    return eventtime + buffer_time - BUFFER_TIME_LOW
                # Under ran low buffer mark - flush lookahead queue
                self._flush_lookahead()
                if print_time != self.print_time:
                    self.check_stall_time = self.print_time
            # In "NeedPrime"/"Priming" state - flush queues if needed
            while 1:
                end_flush = self.need_flush_time + BGFLUSH_EXTRA_TIME
                if self.last_flush_time >= end_flush:
                    self.do_kick_flush_timer = True
                    return self.reactor.NEVER
                buffer_time = self.last_flush_time - est_print_time
                if buffer_time > BGFLUSH_LOW_TIME:
                    return eventtime + buffer_time - BGFLUSH_LOW_TIME
                ftime = est_print_time + BGFLUSH_LOW_TIME + BGFLUSH_BATCH_TIME
                self._advance_flush_time(min(end_flush, ftime))
        except:
            logging.exception("Exception in flush_handler")
            self.printer.invoke_shutdown("Exception in flush_handler")
        return self.reactor.NEVER

    # Movement commands
    def get_position(self, axes:str=None):
        """Returns the position vector of the toolhead.
        Args:
            axes (str, optional): A string indicating which axes to return (e.g. "XYE" for X, Y and E). Defaults to None.
        """
        if axes is None:
            return list(self.commanded_pos)
        else:
            return self.get_axes(self.commanded_pos, axes)

    def get_axes(self, pos: list, axes: str):
        """Subsets a toolhead position vector by axes letters.

        Args:
            axes (str): A string indicating which axes to return (e.g. "XYE" for X, Y and E). Defaults to None.
        """
        return [pos[self.axis_map[a]] for a in axes]

    def get_axes_idxs(self, axes: str):
        return [self.axis_map[a] for a in axes]

    def update_axes(self, pos: list, **kwargs):
        """Update values in a position vector by axis letter ID.

        Args:
            pos (list): Toolhead-like position vector.
            kwargs: Pairs of axis letters and values (e.g. X=1, Z=3) which will be updated.
        """
        pos = pos.copy()
        for k, v in kwargs.items():
            # Get the axis' position index.
            a = self.axis_map[k]
            # Update the axis' value.
            pos[a] = v
        return pos

    def axes_to_xyz(self, axes):
        """Convert ABC axis IDs to XYZ IDs (i.e. 3,4,5 to 0,1,2).

        Has no effect on XYZ IDs
        """
        logging.info(f"toolhead.axes_to_xyz: input={axes}")

        xyz_ids = [0, 1, 2, 0, 1, 2]

        try:
            if isinstance(axes, list) or isinstance(axes, tuple):
                result = [xyz_ids[i] for i in axes]
            else:
                result = xyz_ids[axes]
        except:
            raise Exception(f"toolhead.axes_to_xyz: error with input={axes}")

        logging.info(f"toolhead.axes_to_xyz: output={result}")

        return result

    def get_elements(self, toolhead_pos, axes):
        return [toolhead_pos[axis] for axis in axes]

    def make_coords(self, default=None, **kwargs):
        coords = [default for i in range(self.pos_length)]
        for k, v in kwargs.items():
            coords[self.axis_map[k]] = v
        return coords

    def set_position(self, newpos, homing_axes=()):
        logging.info(f"toolhead.set_position: setting newpos={newpos} and homing_axes={homing_axes}")
        self.flush_step_generation()

        # NOTE: Set the position of the axes "trapq".
        for axes in list(self.kinematics):
            # Iterate over["XYZ", "ABC"]
            logging.info(f"toolhead.set_position: setting {axes} trapq position.")
            kin = self.kinematics[axes]
            # Filter the axis IDs according to the current kinematic
            new_kin_pos = self.get_elements(newpos, kin.axis)
            logging.info(f"toolhead.set_position: using newpos={new_kin_pos}")
            self.set_kin_trap_position(kin.trapq, new_kin_pos)

        # NOTE: Also set the position of the extruder's "trapq".
        #       Runs "trapq_set_position" and "rail.set_position".
        logging.info(f"toolhead.set_position: setting E trapq pos.")
        self.set_position_e(newpos_e=newpos[-1], homing_axes=homing_axes)

        # NOTE: Set the position of the axes "kinematics".
        for axes in list(self.kinematics):
            # Iterate over["XYZ", "ABC"]
            logging.info(f"toolhead.set_position: setting {axes} kinematic position.")
            kin = self.kinematics[axes]
            # Filter the axis IDs according to the current kinematic, and convert them to the "0,1,2" range.
            kin_homing_axes = self.axes_to_xyz([axis for axis in homing_axes if axis in kin.axis])
            new_kin_pos = self.get_elements(newpos, kin.axis)
            logging.info(f"toolhead.set_position: using newpos={new_kin_pos} and kin_homing_axes={kin_homing_axes}")
            self.set_kinematics_position(kin=kin, newpos=new_kin_pos, homing_axes=tuple(kin_homing_axes))

        # NOTE: "set_position_e" was inserted above and not after
        #       updating "commanded_pos" under the suspicion that
        #       an unmodified "commanded_pos" might be important.
        self.commanded_pos[:] = newpos

        # NOTE: this event is mainly recived by gcode_move.reset_last_position,
        #       which updates its "self.last_position" with (presumably) the
        #       "self.commanded_pos" above.
        self.printer.send_event(self.event_prefix + "toolhead:set_position")  # "toolhead:set_position"

    def set_kin_trap_position(self, trapq, newpos):
        """Abstraction of trapq_set_position for different sets of kinematics.

        Args:
            trapq (trapq): trapezoidal queue.
            newpos (list): 3-element list with the new positions for the trapq.
        """

        if trapq is not None:
            # NOTE: Set the position of the toolhead's "trapq".
            logging.info(f"toolhead.set_kin_trap_position: setting trapq pos to newpos={newpos}")
            ffi_main, ffi_lib = chelper.get_ffi()
            ffi_lib.trapq_set_position(self.trapq, self.print_time,
                                       newpos[0], newpos[1], newpos[2])
        else:
            logging.info(f"toolhead.set_kin_trap_position: trapq was None, skipped setting to newpos={newpos}")

    def set_kinematics_position(self, kin, newpos, homing_axes):
        """Abstraction of kin.set_position for different sets of kinematics.

        Args:
            kin (kinematics): Instance of a (cartesian) kinematics class.
            newpos (list): 3-element list with the new positions for the kinematics.
            homing_axes (tuple): 3-element tuple indicating the axes that should have their limits re-applied.
        """
        # NOTE: The "homing_axes" argument is a tuple similar to
        #       "(0,1,2)" (see SET_KINEMATIC_POSITION at "force_move.py"),
        #       used to set axis limits by the (cartesian) kinematics.
        # NOTE: Calls "rail.set_position" on each stepper which in turn
        #       calls "itersolve_set_position" from "itersolve.c".
        # NOTE: Passing only the first three elements (XYZ) to this set_position.
        if kin is not None:
            logging.info(f"toolhead.set_kinematics_position: setting kinematic position with newpos={newpos} and homing_axes={homing_axes}")
            kin.set_position(newpos, homing_axes=tuple(homing_axes))
        else:
            logging.info(f"toolhead.set_kinematics_position: kin was None, skipped setting to newpos={newpos} and homing_axes={homing_axes}")

    def set_position_e(self, newpos_e, homing_axes=()):
        """Extruder version of set_position."""
        logging.info(f"toolhead.set_position_e: setting E to newpos={newpos_e}.")

        # Get the active extruder
        extruder: PrinterExtruder = self.get_extruder()  # PrinterExtruder

        if extruder.get_name() is None:
            # Do nothing if the extruder is a "Dummy" extruder.
            pass
        else:
            # NOTE: Let the "extruder kinematic" set its position. This will call
            #       set position on the "trapq" and "rail" objects of the
            #       active ExtruderStepper class
            # TODO: the "homing_axes" parameter is not used rait nau.
            extruder.set_position(newpos_e, homing_axes, self.print_time)

    def move(self, newpos, speed):
        """ToolHead.move() creates a Move() object with the parameters of the move (in cartesian space and in units of seconds and millimeters).

        Args:
            newpos (_type_): _description_
            speed (_type_): _description_
        """

        logging.info(f"toolhead.move: processing move to newpos={newpos} at speed={speed}")

        # Check if any unconfigured (non-extruder) axes are being moved.
        moved_axes = [i for i, (start_pos, end_pos) in enumerate(zip(self.commanded_pos, newpos)) if start_pos != end_pos]
        unconfigured_axes = list(set(moved_axes).difference(self.axes))
        logging.info(f"toolhead.move: moved_axes={moved_axes} unconfigured_axes={unconfigured_axes} self.axes={self.axes}")
        if unconfigured_axes:
            unconfigured_axes_names = "".join( [ list(self.axis_map)[ax] for ax in unconfigured_axes] )
            raise self.printer.command_error(f"Toolhead move: you must configure the {unconfigured_axes_names} axes ({unconfigured_axes}) in order to use them.")

        logging.info(f"toolhead.move: moving to newpos={newpos}")
        move = Move(toolhead=self,
                    start_pos=self.commanded_pos,
                    end_pos=newpos,
                    speed=speed)
        # NOTE: So far, the clock time for when this move
        #       will be sent are not known.
        # NOTE: Stepper move commands are not sent with
        #       a "clock" argument.

        # NOTE: Move checks.
        if not move.move_d:
            logging.info(f"toolhead.move: early return, nothing to move. move.move_d={move.move_d}")
            return

        # NOTE: Kinematic move checks for XYZ and ABC axes.
        #       The check is skipped if the displacement vector is "small"
        #       (and thus is_kinematic_move is False, see the "Move" class above).
        if move.is_kinematic_move:
            # for axes in ["XYZ"]:
            for axes in list(self.kinematics):
                # Iterate over["XYZ", "ABC"]
                logging.info(f"toolhead.move: check_move on {axes} move.")
                kin = self.kinematics[axes]
                kin.check_move(move)
            # self.kin.check_move(move)
            # TODO: implement move checks for ABC axes here too.
            # if self.abc_trapq is not None:
            #     self.kin_abc.check_move(move)

        # NOTE: Kinematic move checks for E axis.
        if move.axes_d[-1]:
            logging.info(f"toolhead.move: check_move on E move with displacement: {move.axes_d[-1]}")
            # NOTE: The extruder will check the move assuming that the last coordinate is the E axis.
            self.extruder.check_move(move)

        # NOTE: Update "commanded_pos" with the "end_pos"
        #       of the current move command.
        self.commanded_pos[:] = move.end_pos

        # NOTE: Add the Move object to the MoveQueue.
        #       This can trigger "_process_moves".
        self.lookahead.add_move(move)
        if self.print_time > self.need_check_pause:
            self._check_pause()

    def manual_move(self, coord, speed):
        # NOTE: the "manual_move" command interprets "None" values
        #       as the latest (commanded) coordinates.

        # NOTE: get the current (last) position.
        curpos = list(self.commanded_pos)

        # NOTE: Update the current position with the move's target postion.
        for i in range(len(coord)):
            if coord[i] is not None:
                curpos[i] = coord[i]

        # NOTE: send move.
        self.move(curpos, speed)

        # NOTE: This event is handled by "reset_last_position"
        #       (at gcode_move.py) which updates "self.last_position"
        #       in the GCodeMove class.
        self.printer.send_event(self.event_prefix + "toolhead:manual_move")  # "toolhead:manual_move"

    def dwell(self, delay):
        # NOTE: get_last_move_time runs "_flush_lookahead" which then
        #       calls "flush" on the MoveQueue, and ends up calling
        #       "_update_move_time", which updates "self.print_time".
        #       In essence "get_last_move_time" returns an updated
        #       "self.print_time". The delay is then added to it.
        next_print_time = self.get_last_move_time() + max(0., delay)
        self._advance_move_time(next_print_time)
        self._check_pause()

    def wait_moves(self):
        # NOTE: This function waits until print_time is in sync with the present
        #       time (i.e. the current time of the host computer). The waiting
        #       is skipped if the toolhead enters a special state.

        # NOTE: Calls "move_queue.flush" unless in "special queuing state"
        #       (e.g. drip mode).
        # TODO: Check if this is the cause of the bug reported at Discord:
        #       https://discord.com/channels/431557959978450984/801826273227177984/1085312803558133800
        #       And fixed by an M400:
        #       https://discord.com/channels/431557959978450984/801826273227177984/1086104085201158260
        self._flush_lookahead()

        # NOTE: See "reactor.py"
        #       "Return the monotonic system time as a double"
        eventtime = self.reactor.monotonic()

        # NOTE: Loop while the queuing state is "regular" (e.g. not "drip"),
        #       or while the "print_time" is greater than the result of
        #       "mcu.estimated_print_time(eventtime)" (which converts "clock time"
        #       to "print time", see "clocksync.py").
        while (not self.special_queuing_state) or (self.print_time >= self.mcu.estimated_print_time(eventtime)):

            # NOTE: break the loop if the toolhead "cannot be paused".
            if not self.can_pause:
                break

            # NOTE: "pause" the reactor for a bit before looping again.
            #       This command does a bunch of undocumented stuff with
            #       greenlet objects, and may use "time.sleep" in some case.
            eventtime = self.reactor.pause(eventtime + 0.100)

    def set_extruder(self, extruder, extrude_pos):
        self.extruder = extruder
        self.commanded_pos[-1] = extrude_pos

    def get_extruder(self):
        return self.extruder

    # Homing "drip move" handling
    def _update_drip_move_time(self, next_print_time):
        # NOTE: called by "_process_moves" when in a "special_queuing_state"
        #       (i.e. when its value is not "" or None).
        flush_delay = DRIP_TIME + STEPCOMPRESS_FLUSH_TIME + self.kin_flush_delay
        while self.print_time < next_print_time:
            # NOTE: "drip_completion.test" is a method from "ReactorCompletion",
            #       but is beyond my understanding and deathwishes for spelunking.
            # NOTE: The "drip_completion" object was created by the "multi_complete"
            #       function at "homing.py", from a list of "wait" objects (returned
            #       by the "MCU_endstop.home_start" method, called during homing).
            # TODO: ask what it is for!
            if self.drip_completion.test():
                # NOTE: this "exception" does nothing, it "passes",
                #       but it is caught at the "drip_move" method,
                #       which runs "move_queue.reset" and "trapq_finalize_moves"
                #       in response. This must be an "alternate" way to break
                #       the while loop. A bit hacky though.
                raise DripModeEndSignal()
            curtime = self.reactor.monotonic()
            est_print_time = self.mcu.estimated_print_time(curtime)
            wait_time = self.print_time - est_print_time - flush_delay
            if wait_time > 0. and self.can_pause:
                # Pause before sending more steps
                self.drip_completion.wait(curtime + wait_time)
                continue

            # Send more steps
            npt = min(self.print_time + DRIP_SEGMENT_TIME, next_print_time)
            self.note_mcu_movequeue_activity(npt + self.kin_flush_delay,
                                             set_step_gen_time=True)
            # NOTE: Call "_advance_move_time" with a small time in the future, updating
            #       "self.print_time", generating steps, calling "trapq_finalize_moves",
            #       and calling "MCU.flush_moves".
            self._advance_move_time(npt)
            # NOTE: because of how "print_time" is updated, the while loop will end
            #       before "self.print_time >= next_print_time" by "MOVE_BATCH_TIME".

    def drip_move(self, newpos, speed, drip_completion):
        self.dwell(self.kin_flush_delay)
        # Transition from "NeedPrime"/"Priming"/main state to "Drip" state
        self.lookahead.flush()
        self.special_queuing_state = "Drip"
        self.need_check_pause = self.reactor.NEVER
        self.reactor.update_timer(self.flush_timer, self.reactor.NEVER)
        self.do_kick_flush_timer = False
        self.lookahead.set_flush_time(BUFFER_TIME_HIGH)
        self.check_stall_time = 0.
        self.drip_completion = drip_completion
        # NOTE: The "drip_completion=all_endstop_trigger" object is
        #       probably made from "reactor.completion" objects.
        # NOTE: the "drip_completion.test" method will be used during
        #       the call to "_update_drip_move_time" during a homing move.

        # Submit move
        try:
            # NOTE: Uses "add_move", to add a move to the "move_queue".
            logging.info("drip_move: sending move to the queue.")
            self.move(newpos, speed)
        except self.printer.command_error as e:
            self.reactor.update_timer(self.flush_timer, self.reactor.NOW)
            self.flush_step_generation()
            raise

        # Transmit move in "drip" mode
        try:
            # NOTE: Summary: because the flush function is called with a
            #       not None "special_queuing_state", the "_process_moves"
            #       call (from "flush") will use "_update_drip_move_time".
            #       That method will raise "DripModeEndSignal" when the result of
            #       "drip_completion.test()" is True, thereby ending the move,
            #       and returning here.
            logging.info("drip_move: flushing move queue / transmitting move.")
            self.lookahead.flush()
        except DripModeEndSignal as e:
            logging.info("drip_move: resetting move queue / DripModeEndSignal caught.")

            # NOTE: deletes al moves in the queue and resets "junction_flush" time.
            self.lookahead.reset()

            # NOTE: Expire all pending moves in every "trapq".
            # NOTE: "trapq_finalize_moves" calls a function in "trapq.c", described as:
            #       - Expire any moves older than `print_time` from the trapezoid velocity queue
            #       - Flush all moves from trapq (in the case of print_time=NEVER_TIME)
            #       I am guessing here that "older" means "with a smaller timestamp",
            #       or "previous". Otherwise it would not make sense.
            for axes in list(self.kinematics):
                # Iterate over ["XYZ", "ABC"].
                kin = self.kinematics[axes]
                logging.info(f"ToolHead.drip_move calling trapq_finalize_moves on axes={axes} free_time=self.reactor.NEVER ({self.reactor.NEVER})")
                self.trapq_finalize_moves(kin.trapq, self.reactor.NEVER, 0)

            # # NOTE: This calls a function in "trapq.c", described as:
            # #       - Expire any moves older than `print_time` from the trapezoid velocity queue
            # #       - Flush all moves from trapq (in the case of print_time=NEVER_TIME)
            # #       I am guessing here that "older" means "with a smaller timestamp",
            # #       otherwise it does not make sense.
            # self.trapq_finalize_moves(self.trapq, self.reactor.NEVER)

            # # NOTE: call trapq_finalize_moves on the ABC exes too.
            # self.trapq_finalize_moves(self.abc_trapq, self.reactor.NEVER)

            # NOTE: the above may be specific to toolhead and not to extruder...
            #       Add an "event" that calls this same method on the
            #       extruder trapq as well.
            #self.printer.send_event("toolhead:trapq_finalize_extruder_drip_moves",
            #                        self.reactor.NEVER, self.extruder.name)
            # NOTE: Alternatively, use the "update_move_time" of the extruder object.
            #       This function calls "trapq_finalize_moves(self.trapq, flush_time)"
            #       on the extruder's trapq.
            # TODO: Whether it will mess with XYZ-only homing or not remains to be tested.
            self.extruder.update_move_time(flush_time=self.reactor.NEVER, clear_history_time=0)

        # Exit "Drip" state
        self.reactor.update_timer(self.flush_timer, self.reactor.NOW)
        # NOTE: logging for tracing activity
        logging.info("drip_move: calling flush_step_generation / exit drip state.")
        # NOTE: the "flush_step_generation" method, which calls:
        #       - "flush", which should do nothing (dine just above, and the queue is empty).
        #       - "reactor.update_timer"
        #       - "move_queue.set_flush_time"
        #       - "_update_move_time"
        # NOTE: Pausing the program here prevented a "residual home move" issue
        #       during homing the extruder with a drip move. The solution was
        #       to also call "trapq_finalize_moves" on the extruder's "trapq"
        #       above, and before the following call to "flush_step_generation".
        self.flush_step_generation()

    # Misc commands
    def stats(self, eventtime):
        max_queue_time = max(self.print_time, self.last_flush_time)
        for m in self.all_mcus:
            m.check_active(max_queue_time, eventtime)
        est_print_time = self.mcu.estimated_print_time(eventtime)
        self.clear_history_time = est_print_time - MOVE_HISTORY_EXPIRE
        buffer_time = self.print_time - est_print_time
        is_active = buffer_time > -60. or not self.special_queuing_state
        if self.special_queuing_state == "Drip":
            buffer_time = 0.
        return is_active, "print_time=%.3f buffer_time=%.3f print_stall=%d" % (
            self.print_time, max(buffer_time, 0.), self.print_stall)
    def check_busy(self, eventtime):
        est_print_time = self.mcu.estimated_print_time(eventtime)
        lookahead_empty = not self.lookahead.queue
        return self.print_time, est_print_time, lookahead_empty

    cmd_GET_STATUS_MSG_help = "Prettyfied output from toolhead's get_status."
    def get_status_msg(self, gcmd):
        curtime = self.printer.get_reactor().monotonic()
        status = self.get_status(curtime)
        gcmd.respond_info(pformat(status))

    def get_status(self, eventtime):
        print_time = self.print_time
        estimated_print_time = self.mcu.estimated_print_time(eventtime)

        # TODO: Update get_status to use info from all kinematics.
        res = dict()
        for kin in self.kinematics.values():
            new = kin.get_status(eventtime)
            res = self.concat_kin_status(prev=res, new=new, kin=kin)

        # NOTE: Include the extruder limits if configured.
        if self.extruder.get_name():
            e_stepper = self.extruder.extruder_stepper
            if e_stepper is not None:
                # NOTE: it is ok to use "get_limit_status" instead of "get_status",
                #       as the information required from the toolhead only concerns
                #       information from axes, and not extruder-specific parameters.
                e_status = e_stepper.get_limit_status(eventtime)
                res = self.concat_kin_status(prev=res, new=e_status, kin=e_stepper)

        # Add the standard properties.
        res.update({ 'print_time': print_time,
                     'stalls': self.print_stall,
                     'estimated_print_time': estimated_print_time,
                     'extruder': self.extruder.get_name(),
                     'position': self.Coord(*self.commanded_pos[:-1], e=self.commanded_pos[-1]),
                     'max_velocity': self.max_velocity,
                     'max_accel': self.max_accel,
                     'minimum_cruise_ratio': self.min_cruise_ratio,
                     'square_corner_velocity': self.square_corner_velocity})
        return res

    def concat_kin_status(self, prev: dict, new: dict, kin):
        # Concatenate homed axes.
        prev.setdefault('homed_axes', "")
        prev['homed_axes'] += new.get('homed_axes', "")

        # Update minimum limits.
        prev.setdefault('axis_minimum', self.Coord())
        if kin.axes_min is not None:
            ref_lims: namedtuple = prev['axis_minimum']
            kin_lims: namedtuple = kin.axes_min
            for axis in kin.axis_names.lower():
                value = getattr(kin_lims, axis)
                ref_lims = ref_lims._replace(**{axis: value})
            prev['axis_minimum'] = ref_lims

        # Update maximum limits.
        prev.setdefault('axis_maximum', self.Coord())
        if kin.axes_max is not None:
            ref_lims: namedtuple = prev['axis_maximum']
            kin_lims: namedtuple = kin.axes_max
            for axis in kin.axis_names.lower():
                value = getattr(kin_lims, axis)
                ref_lims = ref_lims._replace(**{axis: value})
            prev['axis_maximum'] = ref_lims

        # Add the missing properties.
        # WARN: This might be a bit hacky, but ok as long as all
        #       special cases are handled above. If not, information
        #       will be lost without warning and probably cause problems.
        for key, value in new.items():
            # This will not overwite.
            prev.setdefault(key, value)

        return prev

    def _handle_shutdown(self):
        self.can_pause = False
        self.lookahead.reset()

    def get_kinematics(self, axes="XYZ"):
        if axes == "XYZ":
            return self.kinematics[axes]
        elif axes == "ABC":
            return self.kinematics[axes]
        else:
            logging.warning(f"get_kinematics: No kinematics matched to axes={axes} returning 'toolhead.kin' (legacy behaviour).")
            return self.kin
    def get_kinematics_abc(self):
        # TODO: update the rest of the code to use "get_trapq" with "axes" instead.
        return self.kin_abc

    def get_trapq(self, axes="XYZ"):
        if axes == "XYZ":
            return self.kinematics[axes].trapq
        elif axes == "ABC":
            return self.kinematics[axes].trapq
        else:
            logging.info(f"get_trapq: No kinematics matched to axes={axes} returning 'toolhead.trapq' (legacy behaviour).")
            return self.trapq
    def get_abc_trapq(self):
        # TODO: update the rest of the code to use "get_trapq" with "axes" instead.
        return self.abc_trapq

    def register_step_generator(self, handler):
        self.step_generators.append(handler)
    def note_step_generation_scan_time(self, delay, old_delay=0.):
        self.flush_step_generation()
        if old_delay:
            self.kin_flush_times.pop(self.kin_flush_times.index(old_delay))
        if delay:
            self.kin_flush_times.append(delay)
        new_delay = max(self.kin_flush_times + [SDS_CHECK_TIME])
        self.kin_flush_delay = new_delay
    def register_lookahead_callback(self, callback):
        last_move = self.lookahead.get_last()
        if last_move is None:
            callback(self.get_last_move_time())
            return
        last_move.timing_callbacks.append(callback)
    def note_mcu_movequeue_activity(self, mq_time, set_step_gen_time=False):
        self.need_flush_time = max(self.need_flush_time, mq_time)
        if set_step_gen_time:
            self.step_gen_time = max(self.step_gen_time, mq_time)
        if self.do_kick_flush_timer:
            self.do_kick_flush_timer = False
            self.reactor.update_timer(self.flush_timer, self.reactor.NOW)
    def get_max_velocity(self):
        return self.max_velocity, self.max_accel
    def _calc_junction_deviation(self):
        scv2 = self.square_corner_velocity**2
        self.junction_deviation = scv2 * (math.sqrt(2.) - 1.) / self.max_accel
        self.max_accel_to_decel = self.max_accel * (1. - self.min_cruise_ratio)
    cmd_G4_help = "Dwell in milliseconds"
    def cmd_G4(self, gcmd):
        # Dwell
        delay = gcmd.get_float('P', 0., minval=0.) / 1000.
        self.dwell(delay)
    cmd_M400_help = "Wait for current moves to finish"
    def cmd_M400(self, gcmd):
        # Wait for current moves to finish
        self.wait_moves()
    cmd_SET_VELOCITY_LIMIT_help = "Set printer velocity limits"
    def cmd_SET_VELOCITY_LIMIT(self, gcmd):
        max_velocity = gcmd.get_float('VELOCITY', None, above=0.)
        max_accel = gcmd.get_float('ACCEL', None, above=0.)
        square_corner_velocity = gcmd.get_float(
            'SQUARE_CORNER_VELOCITY', None, minval=0.)
        min_cruise_ratio = gcmd.get_float(
            'MINIMUM_CRUISE_RATIO', None, minval=0., below=1.)
        if min_cruise_ratio is None:
            req_accel_to_decel = gcmd.get_float('ACCEL_TO_DECEL',
                                                None, above=0.)
            if req_accel_to_decel is not None and max_accel is not None:
                min_cruise_ratio = 1. - min(1., req_accel_to_decel / max_accel)
            elif req_accel_to_decel is not None and max_accel is None:
                min_cruise_ratio = 1. - min(1., (req_accel_to_decel
                                                 / self.max_accel))
        if max_velocity is not None:
            self.max_velocity = max_velocity
        if max_accel is not None:
            self.max_accel = max_accel
        if square_corner_velocity is not None:
            self.square_corner_velocity = square_corner_velocity
        if min_cruise_ratio is not None:
            self.min_cruise_ratio = min_cruise_ratio
        self._calc_junction_deviation()
        msg = ("max_velocity: %.6f\n"
               "max_accel: %.6f\n"
               "minimum_cruise_ratio: %.6f\n"
               "square_corner_velocity: %.6f" % (
                   self.max_velocity, self.max_accel,
                   self.min_cruise_ratio, self.square_corner_velocity))
        # TODO: Is "self.event_prefix" really neded here?
        self.printer.set_rollover_info("toolhead", self.event_prefix + "toolhead: %s" % (msg,))
        if (max_velocity is None and max_accel is None
            and square_corner_velocity is None and min_cruise_ratio is None):
            gcmd.respond_info(msg, log=False)
    cmd_M204_help = "Set default acceleration."
    def cmd_M204(self, gcmd):
        # Use S for accel
        accel = gcmd.get_float('S', None, above=0.)
        if accel is None:
            # Use minimum of P and T for accel
            p = gcmd.get_float('P', None, above=0.)
            t = gcmd.get_float('T', None, above=0.)
            if p is None or t is None:
                gcmd.respond_info('Invalid M204 command "%s"'
                                  % (gcmd.get_commandline(),))
                return
            accel = min(p, t)
        self.max_accel = accel
        self._calc_junction_deviation()

def add_printer_objects(config):
    config.get_printer().add_object('toolhead', ToolHead(config))
    kinematics.extruder.add_printer_objects(config)
