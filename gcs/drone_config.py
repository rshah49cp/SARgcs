
import csv
import glob
import logging
import math
from geographiclib.geodesic import Geodesic
import navpy
import numpy as np
import requests
from enum import Enum
import os

class GCS_COMMAND(Enum):
        IDLE = 0
        DRONE_SHOW_FROM_CSV = 1
        SMART_SWARM = 2
        TAKE_OFF = 10
        LAND = 101
        HOLD = 102
        TEST = 100

class DroneConfig:
    def __init__(self, drones, _hw_id=None, _master=None):
        self.hw_id = _hw_id
        self.master = _master
        self.trigger_time = 0
        self.config = self.read_config()
        self.swarm = self.read_swarm()
        self.state = 0
        self.pos_id = self.get_hw_id(_hw_id)
        self.mission = 0
        self.trigger_time = 0
        self.position = {'lat': 0, 'long': 0, 'alt': 0}
        self.velocity = {'north': 0, 'east': 0, 'down': 0}
        self.yaw = 0
        self.battery = 0
        self.last_update_timestamp = 0
        self.home_position = None
        self.position_setpoint_LLA = {'lat': 0, 'long': 0, 'alt': 0}
        self.position_setpoint_NED = {'north': 0, 'east': 0, 'down': 0}
        self.velocity_setpoint_NED = {'north': 0, 'east': 0, 'down': 0}
        self.yaw_setpoint=0
        self.target_drone = None
        self.drones = drones
        self.gcs_cmd = 'a'
        self.gcs_cmd_ack = False
        self.rssi = None


    def get_hw_id(self, hw_id=None):
        if hw_id is not None:
            return hw_id

        current_directory = os.getcwd()
        grandparent_directory = os.path.abspath(os.path.join(current_directory, os.pardir, os.pardir))
        hw_id_files = glob.glob(os.path.join(grandparent_directory, '**/*.hwID'), recursive=True)

        if hw_id_files:
            hw_id_file = os.path.basename(hw_id_files[0])
            print(f"Hardware ID file found: {hw_id_file}")
            hw_id = int(hw_id_file.split(".")[0])
            print(f"Hardware ID: {hw_id}")
            return hw_id
        else:
            print("Hardware ID file not found. Please check your files.")
            return None

    def read_file(self, filename, source, hw_id):
        with open(filename, newline='') as csvfile:
            reader = csv.DictReader(csvfile)
            for row in reader:
                if int(row['hw_id']) == int(hw_id):
                    return row
        return None

    def read_config(self):
        return self.read_file('config.csv', 'local CSV file', self.hw_id)

    def read_swarm(self):
        return self.read_file('swarm.csv', 'local CSV file', self.hw_id)

   
