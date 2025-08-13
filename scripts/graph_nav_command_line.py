import inspect
import json
import sys
import traceback

import bosdyn.client.util
from bosdyn.client.lease import LeaseClient, LeaseKeepAlive, ResourceAlreadyClaimedError

from src.navigation import graph_nav
from src.structures.graph_nav_client import GraphNavInterface

cmd_dict = {
    '1': (graph_nav.get_localization_state, "Get localization state."),
    '2':
        (graph_nav.set_initial_localization_fiducial,
            "Initialize localization to the nearest fiducial (must be in sight of a fiducial)."
        ),
    '3':
        (graph_nav.set_initial_localization_waypoint,
            "Initialize localization to a specific waypoint (must be exactly at the waypoint)."
        ),
    '4': (graph_nav.list_graph_waypoint_and_edge_ids,
            "List the waypoint ids and edge ids of the map on the robot."),
    '5': (graph_nav.upload_graph_and_snapshots, "Upload the graph and its snapshots."),
    'nav_to': (graph_nav.navigate_to,
            "Navigate to. The destination waypoint id is the second argument."),
    'nav_route': (graph_nav.navigate_route,
            "Navigate route. The (in-order) waypoint ids of the route are the arguments."),
    'nav_act_constr': (graph_nav.navigate_constrained_route,
            "Navigate route to end where an action is performed at a subset of waypoints under"
            "a constraint."),
    'nav_anchor': (
        graph_nav.navigate_to_anchor,
        "Navigate to in seed frame."),
    'clear': (graph_nav.clear_graph, "Clear the current graph."),
    'v': (graph_nav.view_map, "View uploaded map."),
    'd': (graph_nav.download_metadata, "Downloaded recorded metadata."),
    'nav_circuit': (graph_nav.navigate_eulerian_trail, ""),
    'nav_circuit_ns': (graph_nav.navigate_circuit_no_stop, "")
}
    
def run(gn_cli:GraphNavInterface):
    while 1:
        print("Options:")
        for key, (_, description) in cmd_dict.items():
            print(f"\t({key}):\t{description}")
        print("\t(q):\tExit.")
        try:
            inputs = input('>')
        except NameError:
            pass
        req_type = str.split(inputs)[0]
        if req_type == 'q':
            graph_nav.on_quit()
            break
        if req_type not in cmd_dict:
            print('Request not in the known command dictionary.')
            continue
        try:
            cmd_func = cmd_dict[req_type][0]
            if inspect.signature(cmd_func).parameters > 1: cmd_func(gn_cli, str.split(inputs)[1:])
            else: cmd_func(gn_cli)
        except Exception as e:
            print(e)
            print(traceback.format_exc())

def main():
    sdk = bosdyn.client.create_standard_sdk('gn_cli')
    with open('./configs/auth_config.json', 'r') as auth_file:
        auth_config = json.load(auth_file)
        robot = sdk.create_robot(auth_config['hostname'])
        robot.authenticate(auth_config['username'], auth_config['password'])
    with open('./configs/nav_config.json', 'r') as nav_file:
        nav_config = json.load(nav_file)
        gn_cli = GraphNavInterface(robot, nav_config['graph_nav_config']['upload_filepath'], 
                                                   nav_config['graph_nav_config']['use_gps'])
        lease_client = robot.ensure_client(LeaseClient.default_service_name)
        try:
            with LeaseKeepAlive(lease_client, must_acquire=True, return_at_exit=True):
                try:
                    run(gn_cli)
                    return True
                except Exception as exc:
                    print(exc)
                    print('Graph nav command line client threw an error.')
                    return False
        except ResourceAlreadyClaimedError:
            print(
                'Robot lease currently in use.'
            )
            return False

if __name__ == '__main__':
    if not main():
        sys.exit(1)