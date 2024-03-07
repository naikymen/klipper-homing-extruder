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
    from .homing import PrinterHoming

import logging
import pins
from . import manual_probe

HINT_TIMEOUT = """
If the probe did not move far enough to trigger, then
consider reducing the Z axis minimum position so the probe
can travel further (the Z minimum position can be negative).
"""

class PrinterProbe:
    def __init__(self, config: ConfigWrapper, mcu_probe, mcu_probe_name='probe'):
        # NOTE: "mcu_probe" is of "ProbeEndstopWrapper" class, which is a wrapper for "MCU_endstop".
        self.printer: Printer = config.get_printer()
        self.name = config.get_name()
        self.mcu_probe_name=mcu_probe_name
        self.mcu_probe: ProbeEndstopWrapper = mcu_probe
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
        self.gcode.register_command('PROBE', self.cmd_PROBE,
                                    desc=self.cmd_PROBE_help)
        self.gcode.register_command('QUERY_PROBE', self.cmd_QUERY_PROBE,
                                    desc=self.cmd_QUERY_PROBE_help)
        self.gcode.register_command('PROBE_CALIBRATE', self.cmd_PROBE_CALIBRATE,
                                    desc=self.cmd_PROBE_CALIBRATE_help)
        self.gcode.register_command('PROBE_ACCURACY', self.cmd_PROBE_ACCURACY,
                                    desc=self.cmd_PROBE_ACCURACY_help)
        self.gcode.register_command('Z_OFFSET_APPLY_PROBE',
                                    self.cmd_Z_OFFSET_APPLY_PROBE,
                                    desc=self.cmd_Z_OFFSET_APPLY_PROBE_help)
    # NOTE: Methods...
    def _handle_homing_move_begin(self, hmove):
        if self.mcu_probe in hmove.get_mcu_endstops():
            self.mcu_probe.probe_prepare(hmove)
    def _handle_homing_move_end(self, hmove):
        if self.mcu_probe in hmove.get_mcu_endstops():
            self.mcu_probe.probe_finish(hmove)
    def _handle_home_rails_begin(self, homing_state, rails):
        endstops = [es for rail in rails for es, name in rail.get_endstops()]
        if self.mcu_probe in endstops:
            self.multi_probe_begin()
    def _handle_home_rails_end(self, homing_state, rails):
        logging.info(f"probe._handle_home_rails_end: function triggered.")
        endstops = [es for rail in rails for es, name in rail.get_endstops()]
        if self.mcu_probe in endstops:
            self.multi_probe_end()
    def _handle_command_error(self):
        try:
            self.multi_probe_end()
        except:
            logging.exception("Multi-probe end")
    def multi_probe_begin(self):
        self.mcu_probe.multi_probe_begin()
        self.multi_probe_pending = True
    def multi_probe_end(self):
        if self.multi_probe_pending:
            self.multi_probe_pending = False
            self.mcu_probe.multi_probe_end()
    def setup_pin(self, pin_type, pin_params):
        if pin_type != 'endstop' or pin_params['pin'] != 'z_virtual_endstop':
            raise pins.error("Probe virtual endstop only useful as endstop pin")
        if pin_params['invert'] or pin_params['pullup']:
            raise pins.error("Can not pullup/invert probe virtual endstop")
        return self.mcu_probe
    def get_lift_speed(self, gcmd=None):
        if gcmd is not None:
            return gcmd.get_float("LIFT_SPEED", self.lift_speed, above=0.)
        return self.lift_speed
    def get_offsets(self):
        return self.x_offset, self.y_offset, self.z_offset
    
    def _probe(self, speed):
        toolhead: ToolHead = self.printer.lookup_object('toolhead')
        curtime = self.printer.get_reactor().monotonic()
        
        # TODO: consider removing this check.
        if 'z' not in toolhead.get_status(curtime)['homed_axes']:
            raise self.printer.command_error("Must home before probe")
        
        phoming: PrinterHoming = self.printer.lookup_object('homing')
        pos = toolhead.get_position()

        # NOTE: "self.z_position" is equal to the "min_position"
        #       parameter from the "z_stepper" section.
        #       It is used to override the Z component of the 
        #       current toolhead position, probably to generate
        #       the target coordinates for the homing move.
        z_idx = toolhead.axis_map["Z"]
        pos[z_idx] = self.z_position
        
        try:
            # NOTE: This probe method uses "phoming.probing_move",
            #       passing it "mcu_probe" which is an instance of 
            #       "ProbeEndstopWrapper", a wrapper for the probes'
            #       MCU_endstop object.
            # NOTE: This is in contrast to "phoming.manual_home",
            #       which additionally requires a toolhead object.
            #       It turns out that, if not provided, HomingMove
            #       will get the main toolhead by lookup and use it.
            # NOTE: the method is passed "pos", which is "min_position"
            #       parameter from the "z_stepper" section, and the
            #       current XYE toolhead coordinates (see notes above). 
            epos = phoming.probing_move(self.mcu_probe, pos, speed)

        except self.printer.command_error as e:
            # NOTE: the "fail" logic of the G38 gcode could be
            #       based on this behaviour.
            reason = str(e)
            if "Timeout during endstop homing" in reason:
                reason += HINT_TIMEOUT
            raise self.printer.command_error(reason)
        # get z compensation from axis_twist_compensation
        axis_twist_compensation = self.printer.lookup_object(
            'axis_twist_compensation', None)
        z_compensation = 0
        if axis_twist_compensation is not None:
            z_compensation = (
                axis_twist_compensation.get_z_compensation_value(pos))
        # add z compensation to probe position
        epos[z_idx] += z_compensation
        epos_xyz = toolhead.get_axes(epos, "XYZ")
        self.gcode.respond_info("probe at %.3f,%.3f is z=%.6f" % tuple(epos_xyz))
        return epos[:-1]
    
    def _move(self, coord, speed):
        toolhead: ToolHead = self.printer.lookup_object('toolhead')
        toolhead.manual_move(coord, speed)

    def _calc_mean(self, positions):
        count = float(len(positions))
        return [sum([pos[i] for pos in positions]) / count
                for i in range(3)]

    def _calc_median(self, positions):
        z_sorted = sorted(positions, key=(lambda p: p[2]))
        middle = len(positions) // 2
        if (len(positions) & 1) == 1:
            # odd number of samples
            return z_sorted[middle]
        # even number of samples
        return self._calc_mean(z_sorted[middle-1:middle+1])
    
    # TODO: reuse this for probing on any direction.
    # NOTE: actually... it might be simpler to use the "_probe" method above directly.
    #       This function does a lot more than I need for G38.X.
    def run_probe(self, gcmd):
        # NOTE: this command is called by the "cmd_PROBE" handler immediately.
        speed = gcmd.get_float("PROBE_SPEED", self.speed, above=0.)
        lift_speed = self.get_lift_speed(gcmd)
        sample_count = gcmd.get_int("SAMPLES", self.sample_count, minval=1)
        sample_retract_dist = gcmd.get_float("SAMPLE_RETRACT_DIST",
                                             self.sample_retract_dist, above=0.)
        samples_tolerance = gcmd.get_float("SAMPLES_TOLERANCE",
                                           self.samples_tolerance, minval=0.)
        samples_retries = gcmd.get_int("SAMPLES_TOLERANCE_RETRIES",
                                       self.samples_retries, minval=0)
        samples_result = gcmd.get("SAMPLES_RESULT", self.samples_result)
        must_notify_multi_probe = not self.multi_probe_pending
        if must_notify_multi_probe:
            self.multi_probe_begin()

        # NOTE: gets the current Z position of the toolhead, before probing.
        #       It is used below after the probing move, likely to allow for
        #       repeatedprobing.
        toolhead: ToolHead = self.printer.lookup_object('toolhead')
        th_pos = toolhead.get_position()
        z_idx = toolhead.axis_map["Z"]
        
        retries = 0
        positions = []
        while len(positions) < sample_count:
            # Probe position
            # NOTE: "_probe" returns the fourth element in the "epos" list,
            #       returned by "phoming.probing_move" (which is really the
            #       output from "hmove.homing_move").
            pos = self._probe(speed)
            positions.append(pos)
            # Check samples tolerance
            z_positions = [p[z_idx] for p in positions]
            if max(z_positions) - min(z_positions) > samples_tolerance:
                if retries >= samples_retries:
                    raise gcmd.error("Probe samples exceed samples_tolerance")
                gcmd.respond_info("Probe samples exceed tolerance. Retrying...")
                retries += 1
                positions = []
            # Retract
            if len(positions) < sample_count:
                # NOTE: Is this a "rise" before repeating the move?
                new_z = pos[z_idx] + sample_retract_dist
                movepos = toolhead.update_axes(th_pos, Z=new_z)
                self._move(movepos, lift_speed)
        
        if must_notify_multi_probe:
            self.multi_probe_end()
        # Calculate and return result
        if samples_result == 'median':
            return self._calc_median(positions)
        return self._calc_mean(positions)
    
    # TODO: reuse this for probing on any direction.
    cmd_PROBE_help = "Probe Z-height at current XY position"
    def cmd_PROBE(self, gcmd):
        pos = self.run_probe(gcmd)
        gcmd.respond_info("Result is z=%.6f" % (pos[2],))
        self.last_z_result = pos[2]
    
    cmd_QUERY_PROBE_help = "Return the status of the z-probe"
    def cmd_QUERY_PROBE(self, gcmd):
        toolhead: ToolHead = self.printer.lookup_object('toolhead')
        print_time = toolhead.get_last_move_time()
        res = self.mcu_probe.query_endstop(print_time)
        self.last_state = res
        gcmd.respond_info("probe: %s" % (["open", "TRIGGERED"][not not res],))

    def get_status(self, eventtime):
        return {'name': self.name,
                'last_query': self.last_state,
                'last_z_result': self.last_z_result}

    cmd_PROBE_ACCURACY_help = "Probe Z-height accuracy at current XY position"
    def cmd_PROBE_ACCURACY(self, gcmd):
        speed = gcmd.get_float("PROBE_SPEED", self.speed, above=0.)
        lift_speed = self.get_lift_speed(gcmd)
        sample_count = gcmd.get_int("SAMPLES", 10, minval=1)
        sample_retract_dist = gcmd.get_float("SAMPLE_RETRACT_DIST",
                                             self.sample_retract_dist, above=0.)
        toolhead: ToolHead = self.printer.lookup_object('toolhead')
        x_idx, y_idx, z_idx = toolhead.get_axes_idxs("XYZ")  # NOTE: 0, 1, 2
        pos = toolhead.get_position()
        gcmd.respond_info("PROBE_ACCURACY at X:%.3f Y:%.3f Z:%.3f"
                          " (samples=%d retract=%.3f"
                          " speed=%.1f lift_speed=%.1f)\n"
                          % (pos[x_idx], pos[y_idx], pos[z_idx],
                             sample_count, sample_retract_dist,
                             speed, lift_speed))
        # Probe bed sample_count times
        self.multi_probe_begin()
        positions = []
        while len(positions) < sample_count:
            # Probe position
            pos = self._probe(speed)
            positions.append(pos)
            # Retract
            liftpos = toolhead.make_coords(Z=pos[z_idx] + sample_retract_dist)
            self._move(liftpos, lift_speed)
        self.multi_probe_end()
        # Calculate maximum, minimum and average values
        max_value = max([p[z_idx] for p in positions])
        min_value = min([p[z_idx] for p in positions])
        range_value = max_value - min_value
        avg_value = self._calc_mean(positions)[z_idx]
        median = self._calc_median(positions)[z_idx]
        # calculate the standard deviation
        deviation_sum = 0
        for i in range(len(positions)):
            deviation_sum += pow(positions[i][z_idx] - avg_value, 2.)
        sigma = (deviation_sum / len(positions)) ** 0.5
        # Show information
        gcmd.respond_info(
            "probe accuracy results: maximum %.6f, minimum %.6f, range %.6f, "
            "average %.6f, median %.6f, standard deviation %.6f" % (
            max_value, min_value, range_value, avg_value, median, sigma))

    def probe_calibrate_finalize(self, kin_pos):
        if kin_pos is None:
            return
        toolhead: ToolHead = self.printer.lookup_object('toolhead')
        z_idx = toolhead.axis_map["Z"]
        z_offset = self.probe_calibrate_z - kin_pos[z_idx]
        self.gcode.respond_info(
            "%s: z_offset: %.3f\n"
            "The SAVE_CONFIG command will update the printer config file\n"
            "with the above and restart the printer." % (self.name, z_offset))
        configfile = self.printer.lookup_object('configfile')
        configfile.set(self.name, 'z_offset', "%.3f" % (z_offset,))

    cmd_PROBE_CALIBRATE_help = "Calibrate the probe's z_offset"
    def cmd_PROBE_CALIBRATE(self, gcmd):
        manual_probe.verify_no_manual_probe(self.printer)
        # Get axes indexes.
        toolhead: ToolHead = self.printer.lookup_object('toolhead')
        x_idx, y_idx, z_idx = toolhead.get_axes_idxs("XYZ")  # NOTE: 0, 1, 2
        # Perform initial probe
        lift_speed = self.get_lift_speed(gcmd)
        curpos = self.run_probe(gcmd)
        # Move away from the bed
        self.probe_calibrate_z = curpos[z_idx]
        curpos[z_idx] += 5.
        self._move(curpos, lift_speed)
        # Move the nozzle over the probe point
        curpos[x_idx] += self.x_offset
        curpos[y_idx] += self.y_offset
        self._move(curpos, self.speed)
        # Start manual probe
        manual_probe.ManualProbeHelper(self.printer, gcmd,
                                       self.probe_calibrate_finalize)

    def cmd_Z_OFFSET_APPLY_PROBE(self,gcmd):
        offset = self.gcode_move.get_status()['homing_origin'].z
        configfile = self.printer.lookup_object('configfile')
        if offset == 0:
            self.gcode.respond_info("Nothing to do: Z Offset is 0")
        else:
            new_calibrate = self.z_offset - offset
            self.gcode.respond_info(
                "%s: z_offset: %.3f\n"
                "The SAVE_CONFIG command will update the printer config file\n"
                "with the above and restart the printer."
                % (self.name, new_calibrate))
            configfile.set(self.name, 'z_offset', "%.3f" % (new_calibrate,))
    cmd_Z_OFFSET_APPLY_PROBE_help = "Adjust the probe's z_offset"

# Endstop wrapper that enables probe specific features
class ProbeEndstopWrapper:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.position_endstop = config.getfloat('z_offset')
        self.stow_on_each_sample = config.getboolean(
            'deactivate_on_each_sample', True)
        gcode_macro = self.printer.load_object(config, 'gcode_macro')
        self.activate_gcode = gcode_macro.load_template(
            config, 'activate_gcode', '')
        self.deactivate_gcode = gcode_macro.load_template(
            config, 'deactivate_gcode', '')
        
        # Create an "endstop" object to handle the probe pin
        ppins = self.printer.lookup_object('pins')
        pin = config.get('pin')
        pin_params = ppins.lookup_pin(pin, can_invert=True, can_pullup=True)
        mcu = pin_params['chip']
        self.mcu_endstop = mcu.setup_pin('endstop', pin_params)
        self.printer.register_event_handler('klippy:mcu_identify',
                                            self._handle_mcu_identify)
        # Wrappers
        self.get_mcu = self.mcu_endstop.get_mcu
        self.add_stepper = self.mcu_endstop.add_stepper
        self.get_steppers = self.mcu_endstop.get_steppers
        self.home_start = self.mcu_endstop.home_start
        self.home_wait = self.mcu_endstop.home_wait
        self.query_endstop = self.mcu_endstop.query_endstop
        # multi probes state
        self.multi = 'OFF'
    
    def _handle_mcu_identify(self):
        logging.info("ProbeEndstopWrapper._handle_mcu_identify activated (Z axis)")
        kin = self.printer.lookup_object('toolhead').get_kinematics()
        for stepper in kin.get_steppers():
            if stepper.is_active_axis('z'):
                self.add_stepper(stepper)
    def raise_probe(self):
        toolhead = self.printer.lookup_object('toolhead')
        start_pos = toolhead.get_position()
        self.deactivate_gcode.run_gcode_from_command()
        if toolhead.get_position()[:-1] != start_pos[:-1]:
            raise self.printer.command_error(
                "Toolhead moved during probe activate_gcode script")
    def lower_probe(self):
        toolhead = self.printer.lookup_object('toolhead')
        start_pos = toolhead.get_position()
        self.activate_gcode.run_gcode_from_command()
        if toolhead.get_position()[:-1] != start_pos[:-1]:
            raise self.printer.command_error(
                "Toolhead moved during probe deactivate_gcode script")
    def multi_probe_begin(self):
        if self.stow_on_each_sample:
            return
        self.multi = 'FIRST'
    def multi_probe_end(self):
        if self.stow_on_each_sample:
            return
        self.raise_probe()
        self.multi = 'OFF'
    def probe_prepare(self, hmove):
        if self.multi == 'OFF' or self.multi == 'FIRST':
            self.lower_probe()
            if self.multi == 'FIRST':
                self.multi = 'ON'
    def probe_finish(self, hmove):
        if self.multi == 'OFF':
            self.raise_probe()
    def get_position_endstop(self):
        return self.position_endstop

# Helper code that can probe a series of points and report the
# position at each point.
class ProbePointsHelper:
    def __init__(self, config: ConfigWrapper, finalize_callback, default_points=None):
        self.printer = config.get_printer()
        self.toolhead: ToolHead = None
        self.finalize_callback = finalize_callback
        self.probe_points = default_points
        self.name = config.get_name()
        self.gcode = self.printer.lookup_object('gcode')
        # Read config settings
        if default_points is None or config.get('points', None) is not None:
            self.probe_points = config.getlists('points', seps=(',', '\n'),
                                                parser=float, count=2)
        def_move_z = config.getfloat('horizontal_move_z', 5.)
        self.default_horizontal_move_z = def_move_z
        self.speed = config.getfloat('speed', 50., above=0.)
        self.use_offsets = False
        # Internal probing state
        self.lift_speed = self.speed
        # NOTE: This "probe_offsets" is a default value, that can be
        #       replaced by the offsets from "probe.py", which are 3
        #       (X, Y, and Z) with values from the printer config files.
        # TODO: Consider extending this to multi-axis klipper.
        self.probe_offsets = (0., 0., 0.)
        self.results = []
        # Update toolhead attribute when available.
        self.printer.register_event_handler("klippy:connect",
                                            self._handle_connect)
    def _handle_connect(self):
            self.toolhead = self.printer.lookup_object('toolhead')
    def minimum_points(self,n):
        if len(self.probe_points) < n:
            raise self.printer.config_error(
                "Need at least %d probe points for %s" % (n, self.name))
    def update_probe_points(self, points, min_points):
        self.probe_points = points
        self.minimum_points(min_points)
    def use_xy_offsets(self, use_offsets):
        self.use_offsets = use_offsets
    def get_lift_speed(self):
        return self.lift_speed
    def _move_next(self):
        # Lift toolhead
        speed = self.lift_speed
        if not self.results:
            # Use full speed to first probe position
            speed = self.speed
        movepos = self.toolhead.make_coords(Z=self.horizontal_move_z)
        self.toolhead.manual_move(movepos, speed)
        # Check if done probing
        if len(self.results) >= len(self.probe_points):
            self.toolhead.get_last_move_time()
            res = self.finalize_callback(self.probe_offsets, self.results)
            if res != "retry":
                return True
            self.results = []
        # Move to next XY probe point
        nextpos = list(self.probe_points[len(self.results)])
        x_idx, y_idx = self.toolhead.get_axes_idxs("XY")
        if self.use_offsets:
            nextpos[x_idx] -= self.probe_offsets[x_idx]
            nextpos[y_idx] -= self.probe_offsets[y_idx]
        self.toolhead.manual_move(nextpos, self.speed)
        return False
    def start_probe(self, gcmd):
        manual_probe.verify_no_manual_probe(self.printer)
        # Lookup objects
        # NOTE: The 'probe' object is a "PrinterProbe" instance, that
        #       may have been registered, for example, by "bltouch.py".
        probe: PrinterProbe = self.printer.lookup_object('probe', None)
        method = gcmd.get('METHOD', 'automatic').lower()
        self.results = []
        def_move_z = self.default_horizontal_move_z
        self.horizontal_move_z = gcmd.get_float('HORIZONTAL_MOVE_Z',
                                                def_move_z)
        if probe is None or method != 'automatic':
            # Manual probe
            self.lift_speed = self.speed
            self.probe_offsets = (0., 0., 0.)
            self._manual_probe_start()
            return
        # Perform automatic probing
        self.lift_speed = probe.get_lift_speed(gcmd)
        self.probe_offsets = probe.get_offsets()
        z_idx = self.toolhead.axis_map["Z"]
        if self.horizontal_move_z < self.probe_offsets[z_idx]:
            raise gcmd.error("horizontal_move_z can't be less than"
                             " probe's z_offset")
        probe.multi_probe_begin()
        while 1:
            done = self._move_next()
            if done:
                break
            pos = probe.run_probe(gcmd)
            self.results.append(pos)
        probe.multi_probe_end()
    def _manual_probe_start(self):
        done = self._move_next()
        if not done:
            gcmd = self.gcode.create_gcode_command("", "", {})
            manual_probe.ManualProbeHelper(self.printer, gcmd,
                                           self._manual_probe_finalize)
    def _manual_probe_finalize(self, kin_pos):
        if kin_pos is None:
            return
        self.results.append(kin_pos)
        self._manual_probe_start()

def load_config(config):
    return PrinterProbe(config, ProbeEndstopWrapper(config))
