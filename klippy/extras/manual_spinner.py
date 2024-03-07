# Support for a manual controlled stepper
#
# Copyright (C) 2019-2021  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import stepper, chelper
from . import force_move, manual_stepper
import logging
from queue import Queue, Empty
from threading import Event

class ManualSpinner(manual_stepper.ManualStepper):
    def __init__(self, config):
        self.printer = config.get_printer()
        self.toolhead = None  # NOTE: Set to toolhead on printer handle_ready.
        
        # NOTE: save the "reactor" object, I need it for timers/spinning.
        self.reactor = self.printer.get_reactor()
        
        # NOTE: Need this to register the spin move callback appropriately.
        self.printer.register_event_handler("klippy:ready", self.handle_ready)
        self.spin_timer = None
        
        # Spin speed parameters
        self.spin_speed = 0.0
        self.spinning = 0.0
        
        # Timer delay parameters
        # # TODO: make the spin command delay configurable.
        # Used for running the timer function a bit before the
        # time for the next stepper move.
        self.NEXT_CMD_ANTICIP_TIME = 0.1
        self.DEFAULT_TIMER_DELAY = 1.0
        # self.spin_move_delay = 1.0

        # Command queue and event
        self.cmd_queue = Queue(0)
        self.cmd_event = Event()

        if config.get('endstop_pin', None) is not None:
            self.can_home = True
            # NOTE: Instantiate a new PrinterRail class from the
            #       stepper.py module.
            self.rail = stepper.PrinterRail(
                config, need_position_minmax=False, default_position_endstop=0.)
            self.steppers = self.rail.get_steppers()
        else:
            self.can_home = False
            self.rail = stepper.PrinterStepper(config)
            self.steppers = [self.rail]
        self.velocity = config.getfloat('velocity', 5.0, above=0.0)
        self.accel = self.homing_accel = config.getfloat('accel', 0., minval=0.)
        self.next_cmd_time = 0.

        # Default spin params: MOVE, SPEED, ACCEL, SYNC.
        self.spin_params = (0.0, 0.0, 0.0, 1)
        
        # Setup iterative solver
        ffi_main, ffi_lib = chelper.get_ffi()
        self.trapq = ffi_main.gc(ffi_lib.trapq_alloc(), ffi_lib.trapq_free)
        self.trapq_append = ffi_lib.trapq_append
        self.trapq_finalize_moves = ffi_lib.trapq_finalize_moves
        self.rail.setup_itersolve('cartesian_stepper_alloc', b'x')
        self.rail.set_trapq(self.trapq)
        
        # Register commands
        stepper_name = config.get_name().split()[1]
        gcode = self.printer.lookup_object('gcode')
        gcode.register_mux_command('MANUAL_STEPPER', "STEPPER",
                                   stepper_name, self.cmd_MANUAL_STEPPER,
                                   desc=self.cmd_MANUAL_STEPPER_help)
        gcode.register_mux_command('SPIN_MANUAL_STEPPER', "STEPPER",
                                   stepper_name, self.cmd_SPIN_MANUAL_STEPPER,
                                   desc=self.cmd_SPIN_MANUAL_STEPPER_help)

    # Register timer callback method for the continuous/repeated rotation move.
    def handle_ready(self):
        """Register timer callback for continuous stepper rotation.
        Logic borrowed from "delayed_gcode.py".
        """
        self.toolhead = self.printer.lookup_object('toolhead')
        logging.info(f"manual_stepper.handle_ready: registering self.spin_timer.")
        
        # waketime = self.time_at_print_time()
        waketime = self.reactor.NEVER
        self.spin_timer = self.reactor.register_timer(
            # Callback function.
            self.do_spin_move,
            # Initially the timer should be inactive.
            waketime)

    def time_at_print_time(self, print_time=None):
        # Current system time
        eventtime = self.reactor.monotonic()
        # Current (estimated) MCU print_time
        est_print_time = self.toolhead.mcu.estimated_print_time(eventtime)
        if not print_time:
            # Actual MCU print_time (after the last move)
            print_time = self.toolhead.get_last_move_time()
        # System time just after the last move
        sys_print_time = eventtime + (print_time - est_print_time)
        return sys_print_time

    # Spin GCODE command
    cmd_SPIN_MANUAL_STEPPER_help = "Spin a manually configured stepper continuously"
    def cmd_SPIN_MANUAL_STEPPER(self, gcmd):
        """Rotate continuously"""

        # Wait for other threads: wait for the spin_move callback to count unfinished queue tasks.
        # self.cmd_event.wait()

        # Block waiting threads: prevent the spin_move callback from count unfinished queue tasks yet.
        self.cmd_event.clear()

        # Save parameters
        movedist = gcmd.get_float('MOVE', 360.0)
        speed = gcmd.get_float('SPEED', self.velocity)
        accel = gcmd.get_float('ACCEL', self.accel, minval=0.)
        sync = gcmd.get_int('SYNC', 1)
        spin_params = (movedist, abs(speed), accel, sync)

        # Put the command's data in the command queue.
        self.cmd_queue.put(spin_params, block=False)
        logging.info(f"cmd_SPIN_MANUAL_STEPPER: queuing command with spin_params={spin_params} self.spin_speed={self.spin_speed} self.cmd_queue.unfinished_tasks={self.cmd_queue.unfinished_tasks}")

        # Wake the timer at system_print_time if it is dead.
        if (self.reactor.NEVER == self.spin_timer.waketime):
            system_print_time = self.time_at_print_time()
            self.reactor.update_timer(self.spin_timer, system_print_time)
            logging.info(f"cmd_SPIN_MANUAL_STEPPER: timer dead. Triggering do_spin_move at waketime={system_print_time}.")
        else:
            logging.info(f"cmd_SPIN_MANUAL_STEPPER: timer alive, doing nothing.")

        # Release waiting threads: let the spin_move callback count unfinished tasks.
        self.cmd_event.set()

        # Wake the timer at print_time if it is set to wake up later on
        # (covers "self.spin_timer.waketime == self.reactor.NEVER").
        # system_print_time = self.time_at_print_time()
        # if (system_print_time < self.spin_timer.waketime):
        #     logging.info(f"cmd_SPIN_MANUAL_STEPPER: system_print_time is before waketime, triggering do_spin_move at waketime={system_print_time}.")
        #     self.reactor.update_timer(self.spin_timer, system_print_time)
        # else:
        #     logging.info(f"cmd_SPIN_MANUAL_STEPPER: system_print_time is after waketime, doing nothing.")

        # if (self.cmd_queue.unfinished_tasks == 0) and (not self.spin_speed):
        #     # Wake up the timer function "now" only if the queue is empty,
        #     # and the stepper is not actively spinning.
            
        #     # NOTE: "now" in system-time terms is wrong most contexts.
        #     # self.reactor.update_timer(self.spin_timer, self.reactor.NOW)
            
        #     # Get the estimated system time at the next toolhead print_time.
        #     system_print_time = self.time_at_print_time()  # NOTE: this is a future time.

        #     # If the timer is set to wake up later on, then wake it up earlier.
        #     if system_print_time < self.spin_timer.waketime:
        #         self.reactor.update_timer(self.spin_timer, system_print_time)
        #         logging.info(f"cmd_SPIN_MANUAL_STEPPER: All tasks done. Triggering do_spin_move at waketime={system_print_time}.")
        #     else:
        #         logging.info(f"cmd_SPIN_MANUAL_STEPPER: All tasks done. Triggering do_spin_move at original waketime={self.spin_timer.waketime}.")
            
        # else:
        #     # Let the timer update itself.
        #     logging.info(f"cmd_SPIN_MANUAL_STEPPER: letting do_spin_move trigger itself.")

        # # Put the command's data in the command queue.
        # if (self.cmd_queue.unfinished_tasks == 0) and (not self.spin_speed):
        #     # Wake up the timer function "now" only if the queue is empty,
        #     # and the stepper is not actively spinning.
        #     # self.reactor.update_timer(self.spin_timer, self.reactor.NOW)
        #     waketime = self.time_at_print_time()
        #     logging.info(f"cmd_SPIN_MANUAL_STEPPER: All tasks done. Triggering do_spin_move at waketime={waketime}.")
        #     self.reactor.update_timer(self.spin_timer, waketime)
        #     self.cmd_queue.put(spin_params, block=False)
        # else:
        #     # Let the timer update itself.
        #     logging.info(f"cmd_SPIN_MANUAL_STEPPER: letting do_spin_move trigger itself.")
        #     self.cmd_queue.put(spin_params, block=False)

        # # If the speed is not set, then the stepper must be idling.
        # if not self.spin_speed:
        #     # To have start turning, first update the speed.
        #     self.spin_speed = speed
        #     # Then wake up the timer, which will then call the "do_spin_move" callback.
        #     # However do this only if the startup sequence has completed,
        #     # else wait until the timer runs by itself.
        #     if not self.spinning:
        #         self.reactor.update_timer(self.spin_timer, self.reactor.NOW)
        # else:
        #     # If the stepper is already moving, "do_spin_move" will 
        #     # be called automatically. Just update the speed with 
        #     # the new one.
        #     self.spin_speed = speed


    def cmd_queue_get(self):
        """Get parameters of a GCODE command from the queue"""
        try:
            return self.cmd_queue.get_nowait()
        except Empty:
            # The "Empty" exception is raised of no command
            # was in the queue at this time. Handle it.
            return None

    # Continuous rotation (move repeat) timer callback function.
    def do_spin_move(self, eventtime):

        # Actual MCU print_time (after the last move)
        print_time = self.toolhead.get_last_move_time()
        # Current (estimated) MCU print_time
        est_print_time = self.toolhead.mcu.estimated_print_time(eventtime)

        # Get a command from the queue (put there by "cmd_SPIN_MANUAL_STEPPER").
        new_spin_params = self.cmd_queue_get()
        old_speed = self.spin_params[1]

        # Verbooooseeee
        logging.info(f"do_spin_move: called at eventtime={eventtime} est_print_time={est_print_time} with next_cmd_time={self.next_cmd_time} toolhead.print_time={print_time} new_spin_params={new_spin_params} old_speed={old_speed}")
        
        # If a new command is available, get the updated speed.
        if new_spin_params:
            # Get the new spin parameters.
            self.spin_params = new_spin_params
            # In this case, code below will surely do
            # either a startup, cruise, or break move.
            # new_speed = self.spin_params[1]
        else:
            # In this case, code below will either do
            # a "cruise move" or nothing, depending on
            # the current value of "self.spin_speed".
            # new_speed = None
            pass
        
        # # If no command was available, wait for a bit.
        # if spin_params is None:
        #     # Try again in "self.DEFAULT_TIMER_DELAY" seconds.
        #     return eventtime + self.DEFAULT_TIMER_DELAY

        # Set a default waketime for this function.
        # The default is to not run the timer again automatically.
        waketime = self.reactor.NEVER

        # Get the print_time (MCU time) associated to
        # the timer's "present" event time time (system time).
        est_event_print_time = self.toolhead.mcu.estimated_print_time(eventtime)

        # NOTE: On the different times here:
        #       -   self.next_cmd_time is in "print time" seconds (i.e. MCU time).
        #       -   self.reactor.monotonic is in "system time" seconds (i.e. the Pi's time).
        
        # NOTE: The "self.spin_speed" variable is only updated by the
        # "cmd_SPIN_MANUAL_STEPPER" command.
        
        # Start move (accelerate).
        if (not self.spin_speed) and self.spin_params[1]:
            # If the previous speed is 0 (or None) but 
            # the requested speed is not 0 (or None), 
            # then queue rotation startup commands.
            logging.info(f"do_spin_move: startup with self.spin_params={self.spin_params}")
            
            # Calculate move coordinates from the last commanded position
            movedist = self.spin_params[0]  # MOVE
            startup_dist = self.break_dist(self.spin_params[1], self.spin_params[2])  # SPEED, ACCEL
            cruise_dist = movedist-startup_dist
            
            # Do start/cruise moves.
            starttime = self.do_start(speed=self.spin_params[1],  # SPEED
                                      accel=self.spin_params[2],  # ACCEL
                                      sync=self.spin_params[3])   # SYNC
            
            # TODO: handle negative displacement with copysign.
            if cruise_dist > 0.0:
                crustime = self.do_cruise(dist=cruise_dist, 
                                        speed=self.spin_params[1],  # SPEED
                                        sync=self.spin_params[3])   # SYNC
            else:
                crustime = 0.0
            
            # Calculate the time the current move takes,
            # directly from the "do_x" functions.
            movetime = starttime + crustime
            
            # Calculate the time the current move takes,
            # based on the value of self.next_cmd_time, 
            # updated by "gen_stps_fin_moves".
            # movetime = self.next_cmd_time - est_event_print_time
            
            # Add a fraction of the move duration to the current system time (actually
            # the time at which this callback was triggered).
            # In this way the next move will be queued during the current move
            # (i.e. before it finishes) to avoid sudden stops between them.
            # waketime = eventtime + time_to_next_move*0.2

            # Calculate the time between this timer event triggered (est_event_print_time),
            # and the time of the move's physical execution happens (self.next_cmd_time).
            # The move will happen "time_to_next_move" seconds after 
            # the current event's trigger time (in system units).
            time_to_next_move = self.next_cmd_time - est_event_print_time
            # This can be used to set the waketime a bit before "time_to_next_move" is reached.
            movetime = time_to_next_move
            
            # Add a fraction of the move duration to the current system time (actually
            # the time at which this callback was triggered).
            # In this way the next move will be queued at the midpoint of the cruise
            # move that was just queued.
            # waketime = eventtime + movetime - crustime*0.5

            # Try waking up earlier if the move is very short.
            if movetime < self.NEXT_CMD_ANTICIP_TIME:
                waketime = eventtime + movetime/2
            else:    
                waketime = eventtime + movetime - self.NEXT_CMD_ANTICIP_TIME
            
            # waketime = eventtime + movetime  # This works but cannot be stopped immediately.
            # waketime = self.reactor.monotonic() + self.spin_move_delay
            # waketime = self.reactor.monotonic() + self.next_cmd_time
            
            # Track current status: mark the acceleration sequence as done.
            self.spin_speed = self.spin_params[1]
        
        # Cruise move (sustain motion).
        elif self.spin_speed and self.spin_params[1]:
            # If the requested speed is not 0, and the stepper is "spinning", cruise.
            # NOTE: This step is triggered "~crustime/2" seconds before the end of the 
            #       startup move.
            logging.info(f"do_spin_move: cruising with self.spin_params={self.spin_params}")

            # Do the move
            if self.spin_speed == self.spin_params[1]:
                crustime = self.do_cruise(dist=self.spin_params[0],   # MOVE
                                          speed=self.spin_params[1],  # SPEED
                                          sync=self.spin_params[3])   # SYNC
            else:
                # A speed change was requested
                crustime = self.do_cruise_change(dist=self.spin_params[0],     # MOVE 
                                                 speed_i=self.spin_speed,      # SPEED (old)
                                                 speed_f=self.spin_params[1],  # SPEED (new)
                                                 accel=self.spin_params[2],    # ACCEL
                                                 sync=self.spin_params[3])     # SYNC
                                
            
            # Calculate the time the current move will take,
            # directly from the "do_x" functions.
            movetime = crustime

            # Calculate the time the current move takes,
            # based on the value of self.next_cmd_time, 
            # updated by "gen_stps_fin_moves".
            # movetime = self.next_cmd_time - est_event_print_time
            
            # Add the move duration to the current system time.
            # In this way the next move will be issued during the current move
            # but also before it finishes.
            # waketime = eventtime + time_to_next_move*0.8
            
            # Calculate the time between this timer event triggered,
            # and the time of the move's physical execution happens.
            # The move will happen "time_to_next_move" seconds after 
            # the event's time (in system units).
            time_to_next_move = self.next_cmd_time - est_event_print_time
            movetime = time_to_next_move

            # Add a fraction of the move duration to the current system time (actually
            # the time at which this callback was triggered).
            # In this way the next move will be queued at the midpoint of the cruise
            # move that was just queued.
            # waketime = eventtime + movetime - crustime*0.5
            # waketime = eventtime + movetime*0.5

            # Try waking up earlier if the move is very short.
            if movetime < self.NEXT_CMD_ANTICIP_TIME:
                # waketime = self.reactor.NOW
                waketime = eventtime + movetime/2
            else:    
                waketime = eventtime + movetime - self.NEXT_CMD_ANTICIP_TIME

            # Update the speed
            self.spin_speed = self.spin_params[1]

            # Queue another cruise if none are left
            # TODO: this might not be "thread-safe". This cruise could be queued after a brake GCODE command, causing trouble.
            # if self.cmd_queue.unfinished_tasks == 0:
            #     logging.info(f"do_spin_move: no tasks left, queuing a cruise with self.spin_params={self.spin_params}")
            #     self.cmd_queue.put(self.spin_params, block=False)
        
        # Brake move (decelerate).
        elif self.spin_speed and (not self.spin_params[1]):
            # If the requested speed is zero, but
            # the state is still "spinning", then
            # signal a breaking move.
            logging.info(f"do_spin_move: breaking with self.spin_params={self.spin_params}")

            # Calculate breaking distance.
            initial_speed = old_speed           # Old SPEED, from previous self.spin_params[1]
            break_accel = self.spin_params[2]   # ACCEL
            break_dist = self.break_dist(initial_speed, break_accel) 
            
            # Calculate remaining cruising distance.
            movedist = self.spin_params[0]      # MOVE
            cruise_dist = movedist - break_dist
            
            # Cruise before the break (if necessary).
            # TODO: handle negative displacement with copysign.
            if cruise_dist > 0.0:
                crustime = self.do_cruise(dist=cruise_dist, 
                                          speed=initial_speed,
                                          sync=self.spin_params[3])
            else:
                crustime = 0.0
            
            # Do the break
            brketime = self.do_break(speed=initial_speed,
                                     decel=break_accel,
                                     sync=self.spin_params[3])
            
            # Calculate the time the current move takes,
            # directly from the "do_x" functions.
            movetime = brketime + crustime

            # Calculate the time the current move takes.
            # movetime = self.next_cmd_time - est_event_print_time
            
            # Add a large fraction of it to the current system time.
            # In this way the next move will be issued during the current move
            # but also before it finishes.
            # waketime = eventtime + movetime*0.8
            # waketime = eventtime + movetime*1.0

            # Wait for a command to be added to the queue if it has just arrived.
            # NOTE: might offer some thread safety.
            self.cmd_event.wait()
            
            # Set the next waketime after a brake.
            # TODO: this might not be thread-safe with the GCODE callback,
            #       if the task is queued after this check and before
            #       the waketime is returned (below) and updated.
            if self.cmd_queue.unfinished_tasks <= 1:
                # If only this task (or for ome reason no task) is left,
                # then wake up for doomsday.
                waketime = self.reactor.NEVER
            else:
                # If more tasks have been put in the queue, 
                # wake up at print time or perhaps "NOW".
                # Or perhaps leave it at the "current" waketime.
                # waketime = self.time_at_print_time()
                waketime = self.reactor.NOW
                # waketime = self.spin_timer.waketime

            # Signal break complete
            self.spin_speed = 0.0

        else:
            logging.info(f"do_spin_move: NO CONDITION MATCHED with self.spin_speed={self.spin_speed} self.spin_params={self.spin_params}")    
        
        # Mark the current queue task as done,
        # only if new spin parameters arrived.
        if new_spin_params:
            self.cmd_queue.task_done()

        # Update the timer's next firing time.
        logging.info(f"do_spin_move: function ended with waketime={waketime}")
        return waketime

    def break_dist(self, speed, accel):
        """Calculate acceleration/breaking distance from speed and acceleration."""
        # See: https://en.wikipedia.org/wiki/Acceleration#Uniform_acceleration
        break_distance = (speed**2) / (2*accel)
        # break_time = speed / accel
        return break_distance

    def do_start(self, speed, accel, sync=True):
        logging.info(f"do_start: called with accel={accel} and speed={speed}")

        self.sync_print_time()
        cp = self.rail.get_commanded_position()

        # Acceleration parameters
        accel_t, cruise_t, decel_t = speed / accel, 0.0, 0.0
        start_v, cruise_v, accel = 0.0, 0.0, accel
        
        # Calculate total move time
        movetime = accel_t + cruise_t + decel_t

        # Append move to the trapq
        self.trapq_append_move(
            print_time=self.next_cmd_time, start_pos_x=cp,
            accel_t=speed / accel, accel=accel)

        # Increment "self.next_cmd_time", call "generate_steps" and "trapq_finalize_moves".
        self.gen_stps_fin_moves(movetime, sync)

        logging.info(f"do_start: sent at next_cmd_time={self.next_cmd_time} and movetime={movetime}")
        return movetime

    def do_cruise(self, dist, speed, sync=True):
        logging.info(f"do_cruise: called with dist={dist} and speed={speed}")

        # self.sync_print_time()
        cp = self.rail.get_commanded_position()
        
        # Cruising parameters
        accel_t, cruise_t, decel_t = 0.0, dist / speed, 0.0
        start_v, cruise_v, accel = speed, speed, 0.0

        # Calculate total move time
        movetime = accel_t + cruise_t + decel_t

        # Append move to the trapq
        # TODO: the split of appends did not have an obvious effect.
        n_splits=8
        for i in range(n_splits):
            self.trapq_append_move(
                print_time=self.next_cmd_time + i*(movetime/n_splits),
                start_pos_x=cp + i*(dist/n_splits), cruise_t=(dist / speed)  / (n_splits),
                start_v=speed, cruise_v=speed)

        # Increment "self.next_cmd_time", call "generate_steps" and "trapq_finalize_moves".
        self.gen_stps_fin_moves(movetime, sync)

        logging.info(f"do_cruise: sent at next_cmd_time={self.next_cmd_time} and movetime={movetime}")
        return movetime

    def do_cruise_change(self, dist, speed_i, speed_f, accel, sync=True):
        logging.info(f"do_cruise_change: called with dist={dist} and speed_i={speed_i} speed_f={speed_f}")

        # self.sync_print_time()
        cp = self.rail.get_commanded_position()
        
        # Acceleration parameters
        accel_t = abs(speed_f-speed_i)/accel
        accel_d = accel_t*(speed_f+speed_i)/2
        
        # Cruising parameters
        cruise_d = dist-accel_d
        cruise_t = cruise_d/speed_f
        # No cruising if acceleration is slow and covers the whole move
        if cruise_d < 0.0:
            cruise_d, cruise_t = 0.0, 0.0
        
        # Append move to the trapq
        if speed_f > speed_i:
            # Speeding up
            self.trapq_append_move(print_time=self.next_cmd_time, start_pos_x=cp,
                                   start_v=speed_i, cruise_v=speed_f, accel=accel,
                                   accel_t=accel_t, cruise_t=cruise_t)
            
        elif speed_f < speed_i:
            # Slowing down
            self.trapq_append_move(print_time=self.next_cmd_time, start_pos_x=cp,
                                   start_v=speed_i, cruise_v=speed_i, accel=accel,
                                   cruise_t=cruise_t, decel_t=accel_t)
            
        # Calculate total move time
        movetime = accel_t + cruise_t  # + decel_t

        # Increment "self.next_cmd_time", call "generate_steps" and "trapq_finalize_moves".
        self.gen_stps_fin_moves(movetime, sync)

        logging.info(f"do_cruise: sent at next_cmd_time={self.next_cmd_time} and movetime={movetime}")
        return movetime

    def do_break(self, speed, decel, sync=True):
        logging.info(f"do_break: called with decel={decel} and speed={speed}")

        # self.sync_print_time()
        cp = self.rail.get_commanded_position()

        # Breaking parameters
        accel_t, cruise_t, decel_t = 0.0, 0.0, (speed / decel)
        start_v, cruise_v, accel = speed, 0.0, decel
        # NOTE: Both "start_v" and "cruise_v" must have the value of
        #       the initial speed. If "cruise_v" is set to 0.0, then the
        #       move actually starts from speed 0, and accelerates.

        # Calculate total move time
        movetime = accel_t + cruise_t + decel_t

        # Append move to the trapq
        self.trapq_append_move(
            print_time=self.next_cmd_time, start_pos_x=cp, 
            decel_t= speed/decel, cruise_v=speed, accel=decel)

        # Increment "self.next_cmd_time", call "generate_steps" and "trapq_finalize_moves".
        self.gen_stps_fin_moves(movetime, sync)

        logging.info(f"do_break: sent at next_cmd_time={self.next_cmd_time} and movetime={movetime}")
        return movetime
    
    def gen_stps_fin_moves(self, movetime, sync):
        self.next_cmd_time = self.next_cmd_time + movetime

        # NOTE: In toolhead.py, the (drip) moves use a finalize move time smaller
        #       than the generate step time, which makes sense.
        
        # Calls "itersolve_generate_steps" which "Generates 
        # step times for a range of moves on the trapq" (itersolve.c).
        generate_steps_up_to = self.next_cmd_time
        # generate_steps_up_to = self.next_cmd_time - movetime*0.1  # NOTE: worse step interval peaks
        # generate_steps_up_to = self.next_cmd_time - self.NEXT_CMD_ANTICIP_TIME*0.1  # NOTE: worse step interval peaks
        # generate_steps_up_to = self.next_cmd_time + self.NEXT_CMD_ANTICIP_TIME  # NOTE: stepcompress o=2 i=0 c=39 a=0: Invalid sequence
        # generate_steps_up_to = self.next_cmd_time + 0.005
        # generate_steps_up_to = self.next_cmd_time + movetime*0.1
        self.rail.generate_steps(generate_steps_up_to)
        
        # Expire any moves older than `print_time` from the 
        # trapezoid velocity queue (e.g. flush all moves from 
        # trapq if print_time is very large ~99999.9).
        finalize_moves_up_to = self.next_cmd_time + 99999.9
        # finalize_moves_up_to = self.next_cmd_time  # - movetime*0.01 # stepcompress error
        # finalize_moves_up_to = self.next_cmd_time - self.NEXT_CMD_ANTICIP_TIME*0.1
        # finalize_moves_up_to = self.next_cmd_time - 2*movetime  # NOTE: original self.next_cmd_time, no effect.
        # TODO: Perhaps in this case not all moves should be flushed, as intentended for a "manual_stepper".
        finalize_moves_up_to = self.next_cmd_time - movetime*0.5
        self.trapq_finalize_moves(self.trapq, finalize_moves_up_to)
        
        self.toolhead.note_kinematic_activity(self.next_cmd_time)
        if sync:
            self.sync_print_time()

def load_config_prefix(config):
    return ManualSpinner(config)
