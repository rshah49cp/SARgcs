"""
Author: Alireza Ghaderi
Email: p30planets@gmail.com
GitHub: alireza787b
Repository: github.com/alireza787b/mavsdk_drone_show
Date: June 2023
---------------------------------------------------------------------------

Ground Control Station (GCS) Script for Drone Telemetry and Command Control
---------------------------------------------------------------------------

This script is designed to function as a Ground Control Station (GCS) in a drone network, providing functionality for receiving telemetry data from drones and
sending commands to them. The script communicates with the drones over User Datagram Protocol (UDP), and it can be easily configured and expanded to 
handle multiple drones simultaneously.

Setup and Configuration:
------------------------
The script is set up to read its configuration from a .csv file named 'config.csv' which should be located in the same directory as the script. The columns
in 'config.csv' should be as follows: hw_id, pos_id, x, y, ip, mavlink_port, debug_port, gcs_ip.

Each row in 'config.csv' corresponds to a drone. The 'hw_id' column indicates the hardware ID of the drone, the 'pos_id' gives its position ID, 'x' and 'y'
provide coordinates, 'ip' specifies the drone's IP address, 'mavlink_port' and 'debug_port' provide the ports for MAVLink communication and debug information,
and 'gcs_ip' specifies the IP address of the GCS.

The script supports both single-drone and multiple-drone modes. The mode is determined by the 'single_drone' variable in the script. 
If 'single_drone' is True, the script will function in single-drone mode, where it communicates with a single drone specified by the hardware ID in a file 
named 'i.hwID', where 'i' is the hardware ID of the drone. This file should be located in the same directory as the script.
If 'single_drone' is False, the script operates in multiple-drone mode, communicating with all drones specified in the 'config.csv' file.

Sim Mode:
---------
The script includes a 'sim_mode' boolean variable that allows the user to switch between simulation mode and real-world mode. When 'sim_mode' is set to True,
the script is in simulation mode, and the IP address of the coordinator (essentially the drone control node) is manually defined in the code. 
If 'sim_mode' is False, the script uses the IP address from 'config.csv' for the drones.

Communication Protocol:
-----------------------
The script communicates with the drones over UDP using binary packets. For telemetry data, these packets have the following structure:

- Header (1 byte): A constant value of 77. 
- HW_ID (1 byte): The hardware ID of the drone.
- Pos_ID (1 byte): The position ID of the drone.
- State (1 byte): The current state of the drone.
- Trigger Time (4 bytes): The Unix timestamp when the data was sent.
- Terminator (1 byte): A constant value of 88.

For commands, the packets have a similar structure, but the header value is 55 and the terminator value is 66.

Each part of the packet is packed into a binary format using the struct.pack method from Python's 'struct' library.
The '=' character ensures that the bytes are packed in standard size, 'B' specifies an unsigned char (1 byte), and 'I' specifies an unsigned int (4 bytes).

Example:
--------
For example, a command packet could look like this in binary:

Header = 55 (in binary: 00110111)
HW_ID = 3 (in binary: 00000011)
Pos_ID = 3 (in binary: 

00000011)
State = 1 (in binary: 00000001)
Trigger Time = 1687840743 (in binary: 11001010001100001101101100111111)
Terminator = 66 (in binary: 01000010)

The whole packet in binary would look like this:
00110111 00000011 00000011 00000001 11001010001100001101101100111111 01000010

Please note that the actual binary representation would be a sequence of 8-bit binary numbers.
The above representation is simplified to demonstrate the concept.

Running the script:
-------------------
To run the script, simply execute it in a Python environment. You will be prompted to enter commands for the drone.
You can enter 't' to send a command to the drone or 'q' to quit the script. If you choose to send a command,
you will be asked to enter the number of seconds for the trigger time. During this time, telemetry data will not be printed. 
You can enter '0' to cancel the command and resume printing of telemetry data.

License:
--------
This script is open-source and available to use and modify as per the terms of the license agreement.

Disclaimer:
-----------
The script is provided as-is, and the authors and contributors are not responsible for any damage or loss resulting from its use.
Always ensure that you comply with all local laws and regulations when operating drones.

"""

# Imports
import logging
import struct
import threading
import subprocess
import time
import math
import select
import pandas as pd
import os
from concurrent.futures import ThreadPoolExecutor
from pymavlink import mavutil
from drone_config import DroneConfig

from enum import Enum

class State(Enum):
    IDLE = 0
    ARMED = 1
    TRIGGERED = 2

class Mission(Enum):
    NONE = 0
    DRONE_SHOW_FROM_CSV = 1
    SMART_SWARM = 2


# -------- GLOBAL PARAM INIT -------- #
Radio_Serial_Baudrate = 57600
Pymavlink_Port = 13540

# Setup logger
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Read the config file
config_df = pd.read_csv('config.csv')

# Drones list
drones = {}
drone_count = 0
ack_count = 0
for _, row in config_df.iterrows():
    hw_id = row['hw_id']
    drones[hw_id] = DroneConfig(drones, hw_id)
    drone_count = drone_count + 1


# Threading Init
executor = ThreadPoolExecutor(max_workers=10)

# -------- MAVLINK CONNECTION CONFIG -------- #
try:
    router_config_directory = "gcs_mavlink_conf"
    
    # Clear contents of mavlink router config directory
    if os.path.exists(router_config_directory) and os.path.isdir(router_config_directory):
        files = os.listdir(router_config_directory)    
        for file_name in files:
            file_path = os.path.join(router_config_directory, file_name)
            os.remove(file_path)  
    else:
        os.makedirs(router_config_directory)

    router_procs = []
    Radio_Serial_Port = "ttyUSB0"
        
    # Mavlink Source Config
    logging.info("Radio Connection Through Serial Enabled. Connecting to Radio via Serial")
    mavlink_source = f"/dev/{Radio_Serial_Port}"
    logging.info(f"Using MAVLink source: {mavlink_source}")
    config_content = f"""\
[General]
VERBOSE = True

[UartEndpoint Self]     
Mode = Server
Device = {mavlink_source}
Baud = {Radio_Serial_Baudrate}

[UdpEndpoint GCS]
Mode = Normal
Address = 127.0.0.1
Port = {Pymavlink_Port}

[UdpEndpoint QGC]
Mode = Normal
Address = 127.0.0.1
Port = 14550
"""  
    for hw_id, drone_config in drones.items():
        endpoint_config = f"""
[UdpEndpoint Drone_{drone_config.hw_id}]
Mode = Normal
Address = 127.0.0.1
Port = {Pymavlink_Port + drone_config.hw_id}
"""
    config_content += endpoint_config
          
    # Write Config File
    config_name = f"radio.conf"
    with open(os.path.join(router_config_directory, config_name), "w") as file:
        file.write(config_content)

    # Start Mavlink Router Process
    router_procs.append(subprocess.Popen(f"mavlink-routerd -c {router_config_directory}/{config_name}", shell=True))

    state_tracking_master = mavutil.mavlink_connection(f"udp:localhost:{Pymavlink_Port}")
    
    for hw_id, drone_config in drones.items():
        # Connect Pymavlink Connection
        drone_config.master = mavutil.mavlink_connection(f"udp:localhost:{Pymavlink_Port + drone_config.hw_id}", system = drone_config.hw_id)


except Exception as e:
    logging.error(f"An error occurred in Mavlink Connection Initialize: {e}")


time.sleep(1)

# -------- STATE UPDATE FUNCTIONS -------- # 
def set_drone_config(hw_id, pos_id, state, mission, trigger_time, position, velocity, yaw, battery, last_update_timestamp, rssi):
    drone = drones.get(hw_id)
    if pos_id is not None:
        drone.pos_id = pos_id
    if state is not None:
        drone.state = state
    if mission is not None:
        drone.mission = mission
    if trigger_time is not None:
        drone.trigger_time = trigger_time
    if position is not None:
        drone.position = position
    if velocity is not None:
        drone.velocity = velocity
    if yaw is not None:
        drone.yaw = yaw
    if battery is not None:
        drone.battery = battery
    if last_update_timestamp is not None:
        drone.last_update_timestamp = last_update_timestamp
    if rssi is not None:
        drone.rssi = rssi
    drones[hw_id] = drone


def update_state(drones, data):
    msg_type = data.get_type()
    if msg_type == "STATUSTEXT" or msg_type == "UTM_GLOBAL_POSITION" or msg_type == "ATTITUDE" or msg_type == "SYS_STATUS":
        # Ensures Drone_config object will contain position information - Also helps to filter out non-drone systems
        hw_id = data.get_srcSystem()
        # Create a new instance for the drone if not present
        if hw_id not in drones:
            logging.info(f"Receiving Telemetry from UNKNOWN Drone ID= {hw_id}")
            return

        # Update RSSI Values
        if msg_type == 'STATUSTEXT':
            split_string = data.text.split()
            if split_string[0] == 'RSSI':
                set_drone_config(None, None, None, None, None, None, None, None, None, None, split_string[1])

        # Update Position and Velocity Values
        if msg_type == 'UTM_GLOBAL_POSITION':
            logging.debug(f"Received telemetry from Drone {hw_id}")
            position = {'lat': data.lat / 1E7, 'long': data.lon / 1E7, 'alt': data.alt / 1E7}
            velocity = {'north': data.vx, 'east': data.vy, 'down': data.vz}
            set_drone_config(hw_id, None, None, None, None, position, velocity, None, None, data.time, None)
        
        # Update Yaw Values
        if msg_type == 'ATTITUDE':
            set_drone_config(hw_id, None, None, None, None, None, None, data.yaw, None, None, None)

        # Update Battery Values
        if msg_type == 'SYS_STATUS':
            set_drone_config(hw_id, None, None, None, None, None, None, None, data.voltage_battery, None, None)

def get_drone_state(drones, hw_id):
    drone = drones.get(hw_id)
    if drone is not None:
        drone_state = {
        "hw_id": int(drone.hw_id),
        "pos_id": int(drone.config['pos_id']),
        "state": int(drone.state),
        "mission": int(drone.mission),
        "trigger_time": int(drone.trigger_time),
        "position_lat": drone.position['lat'],
        "position_long": drone.position['long'],
        "position_alt": drone.position['alt'],
        "velocity_north": drone.velocity['north'],
        "velocity_east": drone.velocity['east'],
        "velocity_down": drone.velocity['down'],
        "yaw": drone.yaw,
        "battery_voltage": drone.battery,
        "follow_mode": int(drone.swarm['follow']),
        "update_time": int(drone.last_update_timestamp),
        "RSSI": drone.rssi
        }
        return drone_state

def read_packets(master):
    while True:
        ready = select.select([master.fd], [], [], 0.02)
        if ready[0]:
            msg = master.recv_match()
            if (msg):
                update_state(drones, msg)


def start_state_tracking(master):
    state_update_thread = threading.Thread(target=read_packets, args = (master,))
    state_update_thread.start()
    return state_update_thread


def stop_state_tracking(state_update_thread, executor):
    state_update_thread.join()
    executor.shutdown()



# -------- COMMAND SEND FUNCTIONS -------- #
def check_all_drone_ack(drones, ack_count): # simple loops that checks all drone acks
    for drone in drones.values():
        if drone.gcs_msn_ack is False:
            ack_count = 0 # reset the ack count until all drones acks have arrived
            return False
    ack_count = ack_count + 1
    return True

def send_command(master, mission, drones):
    """
    This function prepares and sends commands.

    :param n: An integer used to compute trigger_time.
    :param master: The pymavlink master through which data will be sent.
    :param coordinator_ip: The IP address of the coordinator.
    :param debug_port: The port used for sending data.
    :param hw_id: The hardware ID.
    :param pos_id: The position ID.
    :param mission: The mission ID.
    :param state: The state value.
    """
    try:
        timer = threading.Timer(5.0, cmd_timeout)
        timer.start()

        while True:
            check_all_drone_ack(drones, ack_count)
            
            # Send the command data
            master.mav.statustext_send(
                mavutil.mavlink.MAV_SEVERITY_INFO,
                f"msn {mission}".encode('utf-8')[:50]
            )

            if (ack_count == drone_count):
                timer.cancel
                break

    except (OSError, struct.error) as e:
        # If there is an OSError or an error in packing the data, log the error
        logger.error(f"An error occurred: {e}")

def cmd_timeout():
    # Emergency Ground All Drones if no ACK is recieved
    while True:
        for drone in drones:
            send_command(drone.master, 101)


# -------- MAIN CODE -------- #
try:
    for hw_id, drone_config in drones.items():
        drones_threads = []
        
        # Start the telemetry thread
        state_update_thread = start_state_tracking(state_tracking_master)

        # Add to the drones_threads
        for hw_id, drone_config in drones.items():
            drones_threads.append((drone_config.master, drone_config.hw_id))

    # Main loop for command input
    mission = 0
    state = 0
    n = 0
    time.sleep(1)
    while True:
        command = input("\n Enter 't' for takeoff, 's' for swarm, 'c' for csv_droneshow, 'l' for land, 'n' for none, 'q' to quit: \n")
        if command.lower() == 'q':
            break
        elif command.lower() == 's':
            mission = 2  # Setting mission to smart_swarm
            n = input("\n Enter the number of seconds for the trigger time (or '0' to cancel): \n")
            if int(n) == 0:
                continue
            state = 1
        elif command.lower() == 'c':
            mission = 1  # Setting mission to csv_droneshow
            n = input("\n Enter the number of seconds for the trigger time (or '0' to cancel): \n")
            if int(n) == 0:
                continue
            state = 1
        elif command.lower() == 'n':
            mission = 0  # Unsetting the mission
            state = 0
            n = 0  # Unsetting the trigger time
        elif command.lower() == 't':
            mission = 10
            n = input("\n Enter the number of seconds for the trigger time (or '0' to cancel): \n") 
            if int(n) == 0:
                continue
        elif command.lower() == 'l':
            mission = 101
            n = input("\n Enter the number of seconds for the trigger time (or '0' to cancel): \n") 
            if int(n) == 0:
                continue       
        else:
            logger.warning("Invalid command.")
            continue

        # Send command to each drone
        for master, hw_id in drones_threads:
            trigger_time = int(time.time()) + int(n)  # Now + n seconds
            send_command(master, mission, drones)

except (ValueError, OSError, KeyboardInterrupt) as e:
    # Catch any exceptions that occur during the execution
    logger.error(f"An error occurred: {e}")
finally:
    # When KeyboardInterrupt happens or an error occurs, stop the telemetry threads
    stop_state_tracking(state_update_thread)

    for master, hw_id in drones_threads:
        # Close the pymavlink connection
        master.close()


logger.info("Exiting the application...")

