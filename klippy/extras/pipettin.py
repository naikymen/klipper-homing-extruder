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
import tempfile

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
        dflt_path = os.path.join(tempfile.gettempdir(), "saved_coordinates.json")
        self.output_file = config.get('output_file', dflt_path)
        # Create the file if it does not exist.
        self.create_coordinate_file()

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
        except Exception as e:
            raise gcmd.error(f"Failed to save position data '{position_data}' with error: {e}")
        else:
            # Respond.
            gcmd.respond_info(f"New position saved: {last_position}")

    def create_coordinate_file(self):
        try:
            # Check if the file exists
            if os.path.exists(self.output_file):
                # Attempt to load JSON data
                with open(self.output_file, 'r', encoding='utf-8') as file:
                    json.load(file)
            else:
                # File doesn't exist; create a new one with an empty list
                with open(self.output_file, 'w', encoding='utf-8') as file:
                    json.dump([], file, indent=4)
        except (json.JSONDecodeError, OSError) as e:
            logging.warning(f"Invalid JSON detected or error reading file {self.output_file}: {e}")
            # Overwrite the file with an empty JSON array
            with open(self.output_file, 'w', encoding='utf-8') as file:
                json.dump([], file, indent=4)

    def write_new_coordinate(self, position_data):
        # r+: open the file for reading and writing, without deleting.
        with open(self.output_file, 'r+', encoding='utf-8') as file:
            coordinates = json.load(file)
            coordinates.append(position_data)
            file.seek(0)  # Go back to the beginning of the file
            file.truncate()  # Clear the file's contents
            json.dump(coordinates, file, indent=4)

def load_config(config):
    return Pipettin(config)
