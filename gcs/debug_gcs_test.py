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
import pandas as pd
import os
from pymavlink import mavutil

from enum import Enum

class State(Enum):
    IDLE = 0
    ARMED = 1
    TRIGGERED = 2

class Mission(Enum):
    NONE = 0
    DRONE_SHOW_FROM_CSV = 1
    SMART_SWARM = 2


serial_mavlink = True
Radio_Serial_Baudrate = 57600
Pymavlink_Port = 13540

telem_struct_fmt = '=BHHBBIddddddddBIB'
command_struct_fmt = '=B B B B B I B'

telem_packet_size = struct.calcsize(telem_struct_fmt)
command_packet_size = struct.calcsize(command_struct_fmt)

# Setup logger
logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__name__)

# Single Drone
single_drone = False  # Set this to True for single drone connection

# Read the config file
config_df = pd.read_csv('config.csv')

# Drones list
drones = []

if single_drone:
    # Read the hardware ID from the '.hwID' file
    hw_id_file = [file for file in os.listdir() if file.endswith('.hwID')][0]  # Assuming there's only one such file
    hw_id = int(hw_id_file.split('.')[0])  # Extract the hw_id

    # Find the configuration for the drone in the 'config.csv' file
    drone_config = config_df.loc[config_df['hw_id'] == hw_id].iloc[0]
    drones = [drone_config]
else:
    # Add all drones from config file to drones list
    drones = [drone for _, drone in config_df.iterrows()]

# Mavlink Router Config
try:
    router_config_directory = "gcs_mavlink_conf"
    
    #CLEAR CONTENTS OF ROUTER CONFIG DIRECTORY
    if os.path.exists(router_config_directory) and os.path.isdir(router_config_directory):
        # Get a list of all files in the directory
        files = os.listdir(router_config_directory)
    
        # Iterate over each file and delete it
        for file_name in files:
            file_path = os.path.join(router_config_directory, file_name)
            os.remove(file_path)
    
    else:
        os.makedirs(router_config_directory)

    router_procs = []
    for drone_config in drones:
        Radio_Serial_Port = drone_config['serial_port']
        Radio_UDP_Port = drone_config['debug_port']
        
        #MAVLINK SOURCE CONFIG
        if serial_mavlink:
            logging.info("Radio Connection Through Serial Enabled. Connecting to Radio via Serial")
            mavlink_source = f"/dev/{Radio_Serial_Port}"
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
Port = {drone_config['debug_port']}

[UdpEndpoint QGC]
Mode = Normal
Address = 127.0.0.1
Port = 14550
"""  
        else:
            logging.info("Radio Connection Through UDP Enabled. Attempting to connect to Radio via UDP")
            mavlink_source = f"127.0.0.1:{Radio_UDP_Port}"

        logging.info(f"Using MAVLink source: {mavlink_source}")
        
        
        #WRITE CONFIG FILE
        config_name = f"drone{drone_config['hw_id']}.conf"
        with open(os.path.join(router_config_directory, config_name), "w") as file:
            file.write(config_content)

        #START MAVLINK ROUTER PROCESS
        router_procs.append(subprocess.Popen(f"mavlink-routerd -c {router_config_directory}/{config_name}", shell=True))

except Exception as e:
    logging.error(f"An error occurred in Mavlink Router Initialize: {e}")


# Function to send commands
def send_command(master, mission):
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
        text = f"FROM GCS {mission}"
        # Send the command data
        master.mav.statustext_send(
            mavutil.mavlink.MAV_SEVERITY_INFO,
            text.encode('utf-8')[:50]
        )

    except (OSError, struct.error) as e:
        # If there is an OSError or an error in packing the data, log the error
        logger.error(f"An error occurred: {e}")



def handle_telemetry(keep_running, print_telemetry, master):
    """
    This function continuously receives and handles telemetry data.

    :param keep_running: A control flag for the while loop. 
                         When it's False, the function stops receiving data.
    :param print_telemetry: A flag to control if the telemetry data should be printed.
    :param master: The pymavlink master from which data will be received.
    """
    while keep_running[0]:
        try:
            # Receive SAR Comms -> Target Localization
            msg = master.recv_match()
            if msg and msg.get_type() == 'STATUSTEXT':
                print(f"ID: {msg.get_srcSystem()} RSSI: {msg.text}")


            # Receive telemetry data -> Agent Localization
            if msg and msg.get_type() == 'UTM_GLOBAL_POSITION':
                print(f"ID: {msg.get_srcSystem()} LAT: {msg.lat} LON: {msg.lon} ALT: {msg.alt}")
                print(f"VE: {msg.vx} VN: {msg.vy} VD: {msg.vz}")

        except (OSError, struct.error) as e:
            # If there is an OSError or an error in unpacking the data, log the error and break the loop
            logger.error(f"An error occurred: {e}")
            break
        


# Drones threads
drones_threads = []

# This flag indicates if the telemetry threads should keep running.
# We use a list so the changes in the main thread can be seen by the telemetry threads.
keep_running = [True]

try:
    for drone_config in drones:
        # Extract variables
        coordinator_ip = drone_config['ip']
        debug_port = int(drone_config['debug_port'])    # Debug port
        gcs_ip = drone_config['gcs_ip']                 # GCS IP
        hw_id = drone_config['hw_id']                   # Hardware ID
        pos_id = drone_config['pos_id']                 # Position ID

        # Log information
        if serial_mavlink:
            logger.info(f"Drone {hw_id} is listening and sending on Port {Radio_Serial_Port} at rate {Radio_Serial_Baudrate}")
        else:
            logger.info(f"Drone {hw_id} is listening and sending on IP {coordinator_ip} and port {debug_port}")

        # Pymavlink Connection
        try:
            master = mavutil.mavlink_connection(f"udp:localhost:{drone_config['debug_port']}", source_system = 99)
        except Exception as e:
            logging.error(f"An error occured Pymavlink Initialize: {e}")

        # This flag controls whether telemetry is printed to the screen. 
        # We use a list so the changes in the main thread can be seen by the telemetry threads.
        print_telemetry = [True]

        # Start the telemetry thread
        telemetry_thread = threading.Thread(target=handle_telemetry, args=(keep_running, print_telemetry, master))
        telemetry_thread.start()

        # Add to the drones_threads
        drones_threads.append((master, telemetry_thread, coordinator_ip, debug_port, hw_id, pos_id))

    # Main loop for command input
    mission = 0
    state = 0
    n = 0
    time.sleep(1)
    while True:
    
    #     text = f"FROM GCS {mission}"
    #     # Send the command data
    #     master.mav.statustext_send(
    #         mavutil.mavlink.MAV_SEVERITY_INFO,
    #         text.encode('utf-8')[:50]
    #     )
    #     time.sleep(1)
        
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

        # Turn off telemetry printing while sending commands
        for _, _, _, _, _, _ in drones_threads:
            print_telemetry[0] = False
        # Send command to each drone
        for master, _, coordinator_ip, debug_port, hw_id, pos_id in drones_threads:
            trigger_time = int(time.time()) + int(n)  # Now + n seconds
            send_command(master, mission)
            # Turn on telemetry printing after sending commands
        for _, _, _, _, _, _ in drones_threads:
            print_telemetry[0] = True
except (ValueError, OSError, KeyboardInterrupt) as e:
    # Catch any exceptions that occur during the execution
    logger.error(f"An error occurred: {e}")
finally:
    # When KeyboardInterrupt happens or an error occurs, stop the telemetry threads
    keep_running[0] = False

    for master, telemetry_thread, _, _, _, _ in drones_threads:
        # Close the pymavlink connection
        master.close()
        # Join the thread
        telemetry_thread.join()

logger.info("Exiting the application...")

