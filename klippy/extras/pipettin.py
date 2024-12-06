"""
Macros for the Pipettin' bot liquid handler.
Copyright (C) 2024  Nicolás A. Méndez <nmendez.ar@gmail.com>
This file may be distributed under the terms of the GNU GPLv3 license.

Add the following sections to enable saving coordinates to a JSON file.

[pipettin]
output_file: /home/pi/printer_data/config/coordinates.json

[gcode_macro SAVE_POINT]
gcode:
    SAVE_COORDINATE
"""

# Type checking without cyclic import error.
# See: https://stackoverflow.com/a/39757388
from __future__ import annotations
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from ..klippy import Printer
    from ..configfile import ConfigWrapper
    from ..toolhead import ToolHead
    from ..gcode import GCodeDispatch, GCodeCommand
    from .gcode_move import GCodeMove
# pylint: disable=missing-class-docstring,missing-function-docstring,invalid-name,line-too-long,consider-using-f-string
# pylint: disable=logging-fstring-interpolation,logging-not-lazy,fixme

import os
import json
import logging

class Pipettin:
    toolhead: ToolHead
    gcode_move: GCodeMove
    def __init__(self, config: ConfigWrapper):
        self.printer: ConfigWrapper = config.get_printer()
        self.printer.register_event_handler('klippy:mcu_identify', self._handle_mcu_identify)
        self.gcode: GCodeDispatch = self.printer.lookup_object('gcode')
        self.gcode.register_command("SAVE_COORDINATE",
                                    self.cmd_SAVE_POINT,
                                    when_not_ready=False,
                                    desc=self.cmd_SAVE_POINT_help)
        # File where coordinates can be saved.
        self.output_file = config.get('output_file', "/tmp/saved_coordinates.json")
        # Create the file if it does not exist.
        if not os.path.exists(self.output_file):
            with open(self.output_file, 'w', encoding='utf-8') as file:
                json.dump([], file, indent=4)

    def _handle_mcu_identify(self):
        self.toolhead: ToolHead = self.printer.lookup_object('toolhead')
        self.gcode_move: GCodeMove = self.printer.lookup_object("gcode_move")

    # Main probe command
    cmd_SAVE_POINT_help = "G38.2 Probe toward workpiece, stop on contact, signal error if failure."
    def cmd_SAVE_POINT(self, gcmd: GCodeCommand):
        # Get the toolhead's last position.
        last_position = self.toolhead.get_position()

        # Get the extruder's name.
        extruder = self.toolhead.get_extruder()
        active_extruder_name = extruder.name

        # Make position data.
        position_data = {
            "position": last_position,
            "extruder": active_extruder_name
        }

        # Save it to the file.
        try:
            self.write_new_coordinate(position_data)
        except Exception:
            raise gcmd.error(f"Failed to save position data: {position_data}")
        else:
            # Respond.
            gcmd.respond_info(f"New position saved: {last_position}")

    def write_new_coordinate(self, position_data):
        try:
            with open(self.output_file, 'w+', encoding='utf-8') as file:
                coordinates = json.load(file)
                coordinates.append(position_data)
                json.dump(coordinates, file, indent=4)
        except Exception as e:
            logging.error(f"Failed to save data: {coordinates}")
            raise e

def load_config(config):
    return Pipettin(config)
