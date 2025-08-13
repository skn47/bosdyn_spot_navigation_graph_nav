import argparse
import inspect
import json
import os
import sys

import bosdyn.client.util
from bosdyn.client.recording import GraphNavRecordingServiceClient

from src.navigation import recording
from src.structures.recording_client import RecordingInterface

cmd_dict = {
    '0': recording.clear_map,
    '1': recording.start_recording,
    '2': recording.stop_recording,
    '3': recording.get_recording_status,
    '4': recording.create_default_waypoint,
    '5': recording.download_full_graph,
    '6': recording.list_graph_waypoint_and_edge_ids,
    '7': recording.create_new_edge,
    '8': recording.create_loop,
    '9': recording.auto_close_loops_prompt,
    'a': recording.optimize_anchoring,
}

def run(r_cli:RecordingInterface):
    while 1:
        print("""
        Options:
        (0) Clear map.
        (1) Start recording a map.
        (2) Stop recording a map.
        (3) Get the recording service's status.
        (4) Create a default waypoint in the current robot's location.
        (5) Download the map after recording.
        (6) List the waypoint ids and edge ids of the map on the robot.
        (7) Create new edge between existing waypoints using odometry.
        (8) Create new edge from last waypoint to first waypoint using odometry.
        (9) Automatically find and close loops.
        (a) Optimize the map's anchoring.
        (q) Exit.
        """)
        try:
            inputs = input('>')
        except NameError:
            pass
        req_type = str.split(inputs)[0]
        if req_type == 'q':
            break
        if req_type not in cmd_dict:
            print('Request not in the known command dictionary.')
            continue
        try:
            cmd_func = cmd_dict[req_type]
            if inspect.signature(cmd_func).parameters > 1: cmd_func(r_cli, str.split(inputs)[1:])
            else: cmd_func(r_cli)
        except Exception as e:
            print(e)

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '-n', '--download_name', help=
        'The name of the directory with the recorded graph and snapshots after it is downloaded',
        default='downloaded_graph'
    )
    options = parser.parse_args()
    sdk = bosdyn.client.create_standard_sdk('r_cli')
    with open('./configs/auth_config.json', 'r') as auth_file:
        auth_config = json.load(auth_file)
        robot = sdk.create_robot(auth_config['hostname'])
        robot.authenticate(auth_config['username'], auth_config['password'])
    with open('./configs/nav_config.json', 'r') as nav_file:
        nav_config = json.load(nav_file)
    session_name = nav_config['recording_config']['recording_session_name']
    if session_name == '':
        session_name = os.path.basename(nav_config['recording_config']['download_name'])
    user_name = nav_config['recording_config']['recording_username']
    if user_name == '':
        user_name = robot._current_user
    client_metadata = GraphNavRecordingServiceClient.make_client_metadata(
        session_name=session_name, client_username=user_name, client_id='client')
    r_cli = RecordingInterface(robot, nav_config['recording_config']['download_filepath'], 
        options.download_name if options.download_name else nav_config['recording_config']['download_name'],
                                                client_metadata, nav_config['recording_config']['use_gps'])
    try:
        run(r_cli)
        return True
    except Exception as exc:
        print(exc)
        print('Recording command line client threw an error.')
        return False

if __name__ == '__main__':
    if not main():
        sys.exit(1)