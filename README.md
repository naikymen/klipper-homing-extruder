# Klipper for CNC 

Welcome to my fork of the Klipper project, with home-able extruders, configurable extra ABC axes, and CNC-style probing!

Full changes and limitations stated further down this readme.

Use cases: as far as I can tell.

- More general CNC usage.
- Syringe extruders.
- Pipetting / liquid-handling robots / lab-automation.

Follow the discussion over at Klipper's forum: https://klipper.discourse.group/t/12249

Critical limitations: you should know this beforehand.

- Only the **cartesian** kinematic has been adapted. Others could be without much work, this is a good place for contributions.
- Acceleration is shared among all axes. Abrupt speed changes in the ABC axes will cause XYZ movements to slow down accordingly.
  - Note: motion on the ABC axes will not affect maximum speed of the XYZ axes, which will match the desired feedrate (`F` parameter).
- Most of the modules in "extra" have not been tested and might not work.
- Limitations stated further down this readme.

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

## Changes and new commands

This fork implements:

- CNC on XYZABCE axes.
    - Module: core modifications to toolhead, homing, and cartesian kinematics. New "abc" cartesian kinematics: [cartesian_abc.py](./klippy/kinematics/cartesian_abc.py)
    - Commands:
        - Move: `G1 X10 Y10 Z10 A10 B10 C10 E10` (same as a regular G1 command).
        - Home: `G28 A` (same as regular `G28`).
        - Multi-probe: `MULTIPROBE2 PROBE_NAME=myprobe A=-20 B=10 F=2` (same as regular `MULTIPROBE`).
        - And so on ...
    - Configuration: add ABC kinematics to `[printer]` and the corresponding `[stepper_abc]` sections (details below).
        - You must use `cartesian_abc` instead of `cartesian` at your `[printer]` section.
        - _Partial_ axis sets are implemented (i.e. `XY`-only for laser machines, `XYZE+A` for cutters, etc.).
    - Limitations:
        - Needs testing on longer GCODE programs. Extra steppers not tested (i.e. `stepper_a1`).
        - XYZ axes seem to throttle during GCODE arcs (see notes below).
    - Notes:
        - The feedrate is shared among all active axes, meaning that an XYA move at F200 will be "slower" in the XY plane than a XY-only move at the same feedrate.
    - Module incompatibilites: probably many. Tested with `virtual_sdcard`, `pause_resume`, and `force_move`. Non-cartesian kinematics are untested and will probably not work, help for porting them is welcome!
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
- G2/G3 arc moves are supported when using ABC axes.
    - Arc move: `G2 X10 Y10 I10 J0 A10 F1000`
    - Note: The extra axis displacements (e.g `A`, `B`, `C`, etc.) are evenly split across the arc segments, just like the extruder and/or helical axes. The final feedrate is adjusted to a higher value if the move involves the extra axes. This is needed because Klipper would otherwise "distribute" the specified feedrate among axes later on according to their relative displacements. This is not expected from a "standard" GCODE Arc move, whose feedrate parameter refers exclusively to the speed of the XYZ axes.
    - Note: for lack of better ideas, the default feedrate for ensuing moves will be reset to the feedrate specified in the arc move, or to the feedrate that was set before the move. This is needed because feedrates are internally adjusted for arc moves involving extra axes (e.g. ABC axes).
    - Limitations: only absolute arc moves are supported (same as upstream Klipper). While feedrate is scaled to compensate for the extra axes, acceleration is not, so you may observe additional "ramping" on the XYZ movement when the extra axes are involved (this is speculative so far).

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

See examples here: https://gitlab.com/pipettin-bot/forks/firmware/klipper-stack/-/tree/pipetting/printer_data/config

These are meant as _soft_ reference configs; you _must_ adjust them to match your setup before using them.

Pin mappings for the Arduino CNC-Shield (v3.0) have been added to this repo: [generic-cnc-shield-v3.0.cfg](./config/generic-cnc-shield-v3.0.cfg)

### ABC axes

Most of the configuration is "stock". Only the `[printer]` section needs slightly different stuff:

- `kinematics`: You must use an `xxx_abc` kinematic. Only the `cartesian_abc` kinematic has been tested.
- `kinematics_abc` (new): The ABC set of axes must also use one of the `xxx_abc` kinematics.
- `axis` (new): This string specifies exactly which axes need to be configured, by their common single-letter name in the CNC world.
  - There must be at least one stepper in the Klipper configuration for each of these letters (e.g. if `axis` contains `X`, there must be a `[stepper_x]` section).
  - Partial specification is allowed for the `cartesian_abc` kinematics (e.g. only `XY` and no `Z`)

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

Then configure the additional ABC steppers, exactly the ones specified in the `axis` parameter. For example, the ABC steppers can be configured just as you would the XYZ:

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

Examples:

- Find [here](https://gitlab.com/pipettin-bot/forks/firmware/klipper-stack/-/tree/pipetting/printer_data/config?ref_type=heads) for the example configs with an A, B or C in their names.

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

### PID sample averaging

This is meant to mitigate the effects of noisy ADCs in Arduinos, with great success. :)

The only new parameter is `samples` which can be added to a `heater_generic` section.

```yaml
[heater_generic well_plate_heater]
# This is the new parameter.
# Set "samples" to an integer value "n". The last "n" measurements will be 
# then used to compute the P term (by averaging) and the D term (by regression).
samples: 10
# The rest of the config is standard stuff.
# See: https://www.klipper3d.org/Config_Reference.html#heater_generic
# ...

```

### Extruder homing

Configure your extruders normally, and then add the required homing parameters. See notes below.

Any number of extruders can be configured to do homing. This requires:

- Adding homing parameters to an `[extruder]` section.
- Adding a `[extruder_home extruder]` section per home-able extruder, replacing `extruder` with the name of the corresponding extruder section.

```yaml
[extruder]
# ...
# To enable homing on the extruder, setup all the 
# usual extruder parameters above, and then add the
# usual homing parameters used by regular steppers:
# See: https://www.klipper3d.org/Config_Reference.html#stepper
position_endstop: 0.0
position_min: 0.0
position_max: 100.0
homing_speed: 25.0
homing_positive_dir: False  # ADJUST TO MATCH YOUR SETUP
endstop_pin: gpio15  # REPLACE WITH THE PIN OF **YOUR** HOMING ENDSTOP

[extruder_home extruder]
# This section is required, but no parameters needed.
```

Usage notes:

- If "homing parameters" are added to an extruder's config, it will need to be homed (or unlocked with `SET_KINEMATIC_POSITION`) before it can be moved, even if the corresponding `[extruder_home extruder]` is not set.
- Note that the `[extruder]` must have an "endstop_pin" defined for it to be home-able. It is otherwise setup as a "regular" extruder, and a corresponding `[extruder_home]` section will not work as exected. For instance, a `HOME_EXTRUDER EXTRUDER=extruder` command fail with this error: `'MCU_stepper' object has no attribute 'get_endstops'`

### Single-probe

Simple enough. For reference, read: https://www.klipper3d.org/Config_Reference.html#probe

```yaml
[probe_G38]
# See: https://www.klipper3d.org/Config_Reference.html#probe
recovery_time: 0.4
pin: gpio19
z_offset: 0
```

Usage notes:
 
- `[probe_G38]` is incompatible with `[probe_G38_multi extruder]`.

### Multi-probing

Note that this module also implements the regular `G38.n` commands added by `[probe_G38]` (hence the incompatibility), using the `[probe_G38_multi]` section associated to the active extruder by name.

For example, this means that:

- `[probe_G38_multi extruder]` will be associated to the main `[extruder]`.
- `[probe_G38_multi]` sections with names that do not match an extruder will only be usable through the `MULTIPROBE` set of commands.

Example configuration:

```yaml
[probe_G38_multi extruder]
# This probe will be associated to the main [extruder] section.
recovery_time: 0.0
pin: ^tools:PC5
z_offset: 0


[probe_G38_multi my_probe]
# This probe will not be associated to an extruder (unless there is one named "my_probe").
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

Thanks to some [changes in upstream moonraker](https://github.com/Arksine/moonraker/issues/615), a properly configured repo can be updated from Mainsail just as the original Klipper.
