# Klipper for CNC 

Welcome to my fork of the Klipper project, with awesome home-able extruders, configurable extra ABC axes, and CNC-style probing!

I also mingled with the PID controller, to _optionally_ mitigate the effect of measurement noise by averaging samples.

Use cases:

- More general CNC usage.
- Syringe extruders.
- Pipetting / liquid-handling robots / lab-automation.

[![Klipper](docs/img/klipper-logo-small.png)](https://www.klipper3d.org/)

---

# Original Klipper docs

https://www.klipper3d.org/

Klipper is a 3d-Printer firmware. It combines the power of a general
purpose computer with one or more micro-controllers. See the
[features document](https://www.klipper3d.org/Features.html) for more information on why you should use Klipper.

To begin using Klipper start by [installing](https://www.klipper3d.org/Installation.html) it.

Klipper is Free Software. See the [license](COPYING) or read the [documentation](https://www.klipper3d.org/Overview.html). We depend on
the generous support from our [sponsors](https://www.klipper3d.org/Sponsors.html).

# Fork notes: 7+axis and more

> Find the associated configuration examples at the following sections.

This fork implements:

- CNC on XYZABCE axes.
    - Module: core modifications to toolhead, homing, and cartesian kinematics. New "abc" cartesian kinematics: [cartesian_abc.py](./klippy/kinematics/cartesian_abc.py)
    - Commands:
        - Move: `G1 X10 Y10 Z10 A10 B10 C10 E10` (same as a regular G1 command).
        - Home: `G28 A` (same as regular `G28`).
        - Multi-probe: `MULTIPROBE2 PROBE_NAME=myprobe A=-20 B=10 F=2` (same as regular `MULTIPROBE`).
        - And so on …
    - Configuration: add ABC kinematics to `[printer]` and the corresponding `[stepper_abc]` sections (details below).
        - Note: you must use `cartesian_abc` instead of `cartesian` at your `[printer]` section.
    - Limitations: needs testing on longer GCODE programs. The 3 stepper sections must be configured, _partial_ extra axis is not implemented yet (i.e. XYZ+AB). Extra steppers not tested (i.e. `stepper_a1`).
    - Module incompatibilites: probably many. Tested with `virtual_sdcard`, `pause_resume`, and `force_move`. Non-cartesian kinematics for the XYZ axes not tested.
- Homing on the steppers of `[extruder]`s.
    - Module: [extruder_home.py](./klippy/extras/extruder_home.py)
    - Command: `HOME_ACTIVE_EXTRUDER`.
    - Mux-command: `HOME_EXTRUDER EXTRUDER=extruder` (will activate `extruder` and reactivate the previous extruder when done).
    - Configuration: add homing parameters to `[extruder]` and add `[extruder_home extruder]` (details below).
    - Note: if "homing parameters" are added to an extruder's config, it will need to be homed (or unlocked with `SET_KINEMATIC_POSITION`) before it can be moved.
    - Limitations: It is untested on extruder steppers configured as `[extruder_stepper]` later synced to a particular `[extruder]`. No "second home" is performed.
- Probing in arbitrary directions with `G38.2`, `G38.3`, `G38.4`, and `G38.5` (single-probe version).
    - Module: [probe_G38.py](./klippy/extras/probe_G38.py)
    - Command: `G38.2 X20 F10`
    - For reference, see [LinuxCNC](http://linuxcnc.org/docs/stable/html/gcode/g-code.html#gcode:g38)'s definition of _G38.n Straight Probe_ commands.
    - Configuration: add a `[probe_G38]` section, specifying the probe pin (details below).
    - Note: affected by `G90`/`G91` and `M82`/ `M83`.
    - Module incompatibilites: `[probe_G38_multi ...]`
- General probing with multiple probe pins is supported by an experimental module:
    - Module: [probe_G38_multi.py](./klippy/extras/probe_G38_multi.py)
    - Command (multiprobe): `MULTIPROBE2 PROBE_NAME=extruder1 Z=-20 F=1` (replace the `2` in `MULTIPROBE2` with `3`, `4`, or `5` for the other probing modes).
    - Command (monoprobe): `G38.2 X20 F10` (replace `.2` by `.3-.5` for the other probing modes). To choose the probe pin, this command will try to match the probe's config name to an extruder name, or fail.
    - Configuration: add a `[probe_G38_multi PROBENAME]` section, specifying the probe pin (details below). If `PROBENAME` matches an extruder's name (e.g. `extruder`) the probe pin remains associated to it. The probe on the active extruder is used for "monoprobe" commands.
    - The probes can be queried with `QUERY_ENDSTOPS` (instead of `QUERY_PROBE`).
    - Note: affected by `G90`/`G91` and `M82`/ `M83`.
    - Known incompatibilites: `[probe_G38]`
- The `SET_KINEMATIC_POSITION` command now works with extruder position as well.
    - Try this out: `SET_KINEMATIC_POSITION E=66.6`
- Absolute extruder moves are now absolute.
    - You can now rely on absolute coordinate systems staying that way unless you update them explicitly (e.g. with `G92 E0` and similar commands).
    - The extruder's coordinate origin used to be altered without warning after every tool-change (i.e. extruder activation) in a way equivalent to sending `G92 E0`. This means that the extruder's origins were effectively relative to the last position of the extruder before a toolchange, which was enforced in Klipper to support the obscure expectations of old slicers.
    - See discussion at: https://klipper.discourse.group/t/6558
- The PID controller now uses sample averaging and linear regression to compute the P and D terms, respectively.
    - This replaces the rather obscure pre-existing logic.
    - This brings much improvement for noisy ADCs, such as the one in my Arduino UNO.
    - The `samples` config parameter must be set to, at least, `2`. I tested it with 5.


## Contributing: Interested in CNC stuff for Klipper?

Not-so-minor modifications to Klippy's core were made to accommodate these features. 

### Pull requests

Pull requests are very welcome over here, and will be merged quickly.

### Buy me a beer

Show your love for this project through Liberapay: https://liberapay.com/naikymen <3

Also consider donating to upstream Klipper and its appendages.

### Chat

Let's chat over here: https://klipper.discourse.group/t/klipper-forks-for-cnc/5698

Cheers!

## Configs

See examples here: [configs-pipetting-bot/configs-mainsail](./config/configs-pipetting-bot/configs-mainsail)

These are meant as reference configs; you _must_ adjust them to match your setup before using them.

### Extra ABC axes config

See examples here: [labo-robot-pinmap-xyzabc](./config/configs-pipetting-bot/configs-mainsail/labo-robot-pinmap-xyzabc)

First configure extra ABC kinematics: [printer.cfg](./config/configs-pipetting-bot/configs-mainsail/labo-robot-pinmap-xyzabc/printer.cfg)

```yaml
[printer]
# Regular pritner configuration.
kinematics: cartesian_abc
# Units: mm/s and mm/s^2:
max_velocity: 5000    # F120000
max_z_velocity: 250   # F30000
max_accel: 1000
# Add ABC kinematics to the toolhead:
kinematics_abc: cartesian_abc
axis: XYZABC
```

Then configure the additional ABC steppers: [printer_steppers_abc.cfg](./config/configs-pipetting-bot/configs-mainsail/labo-robot-pinmap-xyzabc/printer_steppers_abc.cfg)

```yaml
[stepper_a]
# Regular stepper configuration.
# ...

[stepper_b]
# Regular stepper configuration.
# ...

[stepper_c]
# Regular stepper configuration.
# ...
```

What works:

- Movement seems to work :)
- Homing now works.
- Probing with G38 works.
- Limit checks work.

Important TODOs:

- Run tests! Only basic functionality has been covered.
- "Extra" steppers not tested (i.e. `stepper_a1`, etc.)
- `SET_KINEMATIC_POSITION` would _sometimes_ cause `MCU 'tools' shutdown: Rescheduled timer in the past`. I find this error hard to reproduce. Maybe its my UNO's fault. Must track down the cause. See: https://github.com/naikymen/klipper-for-cnc/issues/6
- Consider if it would have been better/simpler to use multiple extruder axes instead of full "cartesian" axes. Adding axes one by one would have been simpler this way. For now, full stepper_a, stepper_b, and stepper_c config sections are mandatory.

### PID sample smoothing config

Have a look at: [heaters.cfg](./config/configs-pipetting-bot/configs-mainsail/labo-robot-pinmap-xyze/heaters.cfg)

This is meant to mitigate the effects of noisy ADCs in Arduinos, with great success :)

```yaml
[thermistor thermistor1]
# A regular thermistor configuration.
temperature1: 24.5
resistance1: 83800
beta: 3950

[heater_generic well_plate_heater]
# This is the new parameter.
# Set "samples" to an integer value "n". The last "n" measurements will be 
# then used to compute the P term (by averaging) and the D term (by regression).
samples: 10
# The rest of the config is standard stuff, left here as an example.
gcode_id: INCUB
heater_pin: heaters:PB0  # D8
max_power: 0.5
sensor_type: thermistor1
sensor_pin: heaters:PC0 # A0
pullup_resistor: 67400    # Using 9930 was far noisier.
smooth_time: 2
control: pid
pid_Kp: 7.0
pid_Ki: 0.008
pid_Kd: 1.0
#pwm_cycle_time:
min_temp: 18
max_temp: 60

[verify_heater well_plate_heater]
# My heater is slow, so the "check_gain_time" value was increased.
check_gain_time: 90
#max_error: 120
#hysteresis: 5
#heating_gain: 2

[idle_timeout]
timeout: 600
gcode:
  M84
# TURN_OFF_HEATERS
# The turn off heaters command was removed here because it interfered with my use case.
# Make sure to restore the defaults if you use a 3d-printer.
```

### Extruder homing config

Main configs:
- [printer.cfg](./config/configs-pipetting-bot/configs-mainsail/labo-robot-pinmap-xyze/printer.cfg)
- [printer_extruder_p20.cfg](./config/configs-pipetting-bot/configs-mainsail/labo-robot-pinmap-xyze/printer_extruder_p20.cfg)
- [printer_extruder_p200.cfg](./config/configs-pipetting-bot/configs-mainsail/labo-robot-pinmap-xyze/printer_extruder_p200.cfg)
- [home_extruder.cfg](./config/configs-pipetting-bot/configs-mainsail/labo-robot-pinmap-xyze/home_extruder.cfg)

Configure your extruders normally, and then add the required homing parameters. See notes below.

At a glance:

```yaml
[extruder]
# ...
# ...
# Setup all the usual extruder parameters before these:
position_endstop: 0.0
position_min: 0.0
position_max: 100.0
homing_speed: 25.0
homing_positive_dir: False  # ADJUST TO MATCH YOUR SETUP
endstop_pin: gpio15  # REPLACE WITH THE PIN OF **YOUR** HOMING ENDSTOP

[extruder_home extruder]
# No parameters needed.

[extruder1]
# ...
# ...
# Setup all the usual extruder parameters before these.
position_endstop: 0.0
position_min: 0.0
position_max: 100.0
homing_speed: 25.0
homing_positive_dir: False  # ADJUST TO MATCH YOUR SETUP
endstop_pin: gpio18  # REPLACE WITH THE PIN OF **YOUR** HOMING ENDSTOP

[extruder_home extruder1]
# No parameters needed.
```

If "homing parameters" are added to an extruder's config, it will need to be homed (or unlocked with `SET_KINEMATIC_POSITION`) before it can be moved, even if the corresponding `[extruder_home extruder]` is not set.

Note that the `[extruder]` must have an "endstop_pin" defined for it to be home-able. It is otherwise setup as a "regular" extruder, and a corresponding `[extruder_home extruder]` section will not work as exected. For instance, a `HOME_EXTRUDER EXTRUDER=extruder` command fail with this error: `'MCU_stepper' object has no attribute 'get_endstops'`

### Single-probe config

Config: [probe_G38.cfg](./config/configs-pipetting-bot/configs-mainsail/labo-robot-pinmap-xyze/probe_G38.cfg)

```yaml
[probe_G38]
recovery_time: 0.4
pin: gpio19
z_offset: 0
```

Note that `[probe_G38]` is incompatible with `[probe_G38_multi extruder]`.

### Multi-probing config

Note that this module also implements the regular `G38.n` commands, using the probe section associated to the active extruder's name. 

Config: [probe_G38_multi.cfg](./config/configs-pipetting-bot/configs-mainsail/labo-robot-pinmap-xyze/probe_G38_multi.cfg)

```yaml
[probe_G38_multi extruder]
recovery_time: 0.0
pin: ^tools:PC5
z_offset: 0


[probe_G38_multi my_probe]
recovery_time: 0.0
pin: ^tools:PB1
z_offset: 0
```

For convenience, their status can show up next to the endstops in Mainsail:

![query_probe_endstops.png](./docs/img/pipetting/query_probe_endstops.png)

# Installation

The easiest way is to use a KIAUH "klipper_repos.txt" file. Details at: https://github.com/th33xitus/kiauh/blob/master/klipper_repos.txt.example

1. SSH into the Pi.
2. Copy "klipper_repos.txt.example" to "klipper_repos.txt".
    - Use the command: `cp kiauh/klipper_repos.txt.example  kiauh/klipper_repos.txt`
4. Edit the `kiauh/klipper_repos.txt` file to append "`naikymen/klipper-for-cnc,pipetting`" after the last line.
    - Use the command: `echo "naikymen/klipper-for-cnc,pipetting" >> kiauh/klipper_repos.txt`
5. Start KIAUH.
    - Use the command: `./kiauh/kiauh.sh`
7. Choose option "`6) [Settings]`".
8. Choose option "`1) Set custom Klipper repository`".
9. Choose the option corresonding to "`naikymen/klipper-for-cnc -> pipetting`"
10. Use KIAUH to uninstall and reinstall Klipper.

## Updates through moonraker

### Using the fork

I've made a Moonraker fork to avoid the issues explained below: https://github.com/naikymen/moonraker-pipetting

Keeing it updated will be cumbersome, but there is no alternative to get everything tidy:

![moonraker_fork.png](./docs/img/pipetting/moonraker_fork.png)

To install, use SSH to replace the moonraker directory. Go to mainsail, restart moonraker, and refresh the "Update Manager". If an "invalid" state appears, click the button and do a "hard reset". Everything should look nice!

### Editing the source

Unfortunately the differing repo and branch names dont play well with the hard-coded stuff in Moonraker.

However by applying this simple diff to `~/moonraker` it will just work:

```diff
diff --git a/moonraker/components/simplyprint.py b/moonraker/components/simplyprint.py
index a3bfac3..ba33057 100644
--- a/moonraker/components/simplyprint.py
+++ b/moonraker/components/simplyprint.py
@@ -1002,7 +1002,8 @@ class SimplyPrint(Subscribable):
             "firmware": "Klipper",
             "firmware_version": version,
             "firmware_date": firmware_date,
-            "firmware_link": "https://github.com/Klipper3d/klipper",
+            # "firmware_link": "https://github.com/Klipper3d/klipper",
+            "firmware_link": "https://github.com/naikymen/klipper-for-cnc",
         }
         diff = self._get_object_diff(fw_info, self.cache.firmware_info)
         if diff:
diff --git a/moonraker/components/update_manager/base_config.py b/moonraker/components/update_manager/base_config.py
index 4bcccbc..7c48aef 100644
--- a/moonraker/components/update_manager/base_config.py
+++ b/moonraker/components/update_manager/base_config.py
@@ -34,7 +34,9 @@ BASE_CONFIG: Dict[str, Dict[str, str]] = {
     },
     "klipper": {
         "moved_origin": "https://github.com/kevinoconnor/klipper.git",
-        "origin": "https://github.com/Klipper3d/klipper.git",
+        # "origin": "https://github.com/Klipper3d/klipper.git",
+        "origin": "https://github.com/naikymen/klipper-for-cnc.git",
+        "primary_branch": "pipetting",
         "requirements": "scripts/klippy-requirements.txt",
         "venv_args": "-p python2",
         "install_script": "scripts/install-octopi.sh",
```

![pipetting_moonraker.png](./docs/img/pipetting_moonraker.png)

Since this modifies the Moonraker source, now Moonraker won't be updatable because it will have diverged and thus "invalid" or "dirty" (rawr?).

![moonraker_pipetting.png](./docs/img/moonraker_pipetting.png)

A price I am willing to pay (?)

Raise your hand here to change this: https://github.com/Arksine/moonraker/issues/615

