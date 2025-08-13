import json
import math
import os
import time

from bosdyn.api import robot_state_pb2
from bosdyn.api.gps import gps_pb2
from bosdyn.api.graph_nav import graph_nav_pb2, map_pb2, nav_pb2
from bosdyn.client.exceptions import ResponseError
from bosdyn.client.frame_helpers import get_odom_tform_body
from bosdyn.client.math_helpers import Quat, SE3Pose
from bosdyn.client.power import power_on_motors, safe_power_off_motors
from bosdyn.client.robot_command import RobotCommandBuilder

from navigation import graph_nav_util
from structures.graph_nav_client import GraphNavInterface
from visualization import mapping

def view_map(gni:GraphNavInterface, constr=0):
    wp_start = get_localization_state(gni).localization.waypoint_id
    if gni.current_adj is None:
        print('Current adjacency list is empty. List the waypoints and edges to update the adjacency list.')
        return
    selected = graph_nav_util.find_satisfying_waypoints(gni.current_adj, wp_start, constr)
    mapping.render_map(gni.upload_filepath, selected)

def get_localization_state(gni:GraphNavInterface):
    state = gni.graph_nav_client.get_localization_state(request_gps_state=gni.use_gps)
    print(f'Got localization: \n{state.localization}')
    odom_tform_body = get_odom_tform_body(state.robot_kinematics.transforms_snapshot)
    print(f'Got robot state in kinematic odometry frame: \n{odom_tform_body}')
    if gni.use_gps:
        print(f'GPS info:\n{state.gps}')

def set_initial_localization_fiducial(gni:GraphNavInterface):
    robot_state = gni.robot_state_client.get_robot_state()
    current_odom_tform_body = get_odom_tform_body(
        robot_state.kinematic_state.transforms_snapshot).to_proto()
    localization = nav_pb2.Localization()
    gni.graph_nav_client.set_localization(initial_guess_localization=localization,
                                          ko_tform_body=current_odom_tform_body)

def clear_graph_and_cache(gni:GraphNavInterface):
    return gni.graph_nav_client.clear_graph_and_cache()

def set_initial_localization_waypoint(gni:GraphNavInterface, wp:str):
    if not wp:
        print('No waypoint specified to initialize to.')
        return
    destination_waypoint = graph_nav_util.find_unique_waypoint_id(
        wp, gni.current_graph, gni.current_annotation_name_to_wp_id)
    if not destination_waypoint:
        return
    robot_state = gni.robot_state_client.get_robot_state()
    current_odom_tform_body = get_odom_tform_body(
        robot_state.kinematic_state.transforms_snapshot).to_proto()
    localization = nav_pb2.Localization()
    localization.waypoint_id = destination_waypoint
    localization.waypoint_tform_body.rotation.w = 1.0
    gni.graph_nav_client.set_localization(
        initial_guess_localization=localization,
        max_distance=0.2,
        max_yaw=20.0 * math.pi / 180.0,
        fiducial_init=graph_nav_pb2.SetLocalizationRequest.FIDUCIAL_INIT_NO_FIDUCIAL,
        ko_tform_body=current_odom_tform_body)

def list_graph_waypoint_and_edge_ids(gni:GraphNavInterface):
    graph = gni.graph_nav_client.download_graph()
    if graph is None:
        print('Empty graph.')
        return
    gni.current_graph = graph
    localization_id = gni.graph_nav_client.get_localization_state().localization.waypoint_id
    data = graph_nav_util.update_waypoints_and_edges(graph, localization_id)
    gni.current_annotation_name_to_wp_id = data[0]
    gni.current_adj = data[1]
    gni.current_waypoints = data[2]
    gni.current_edges = data[3]
    gni.current_metadata = {wp :
        {'order': i,
        'time': gni.current_waypoints[wp].annotations.creation_time.seconds,
        'position' : (gni.current_waypoints[wp].waypoint_tform_ko.position.x,
        gni.current_waypoints[wp].waypoint_tform_ko.position.y,
        gni.current_waypoints[wp].waypoint_tform_ko.position.z),
        'metadata' : None
        }
        for i,wp in enumerate(graph_nav_util.sort(gni.current_graph, gni.current_adj))
    }

def upload_graph_and_snapshots(gni:GraphNavInterface):
    print('Loading the graph from disk into local storage...')
    with open(gni.upload_filepath + '/graph', 'rb') as graph_file:
        data = graph_file.read()
        gni.current_graph = map_pb2.Graph()
        gni.current_graph.ParseFromString(data)
        print(
            f'Loaded graph has {len(gni.current_graph.waypoints)} waypoints and {len(gni.current_graph.edges)} edges'
        )
    for waypoint in gni.current_graph.waypoints:
        gni.current_waypoints[waypoint.id] = waypoint
        with open(f'{gni.upload_filepath}/waypoint_snapshots/{waypoint.snapshot_id}',
                    'rb') as snapshot_file:
            waypoint_snapshot = map_pb2.WaypointSnapshot()
            waypoint_snapshot.ParseFromString(snapshot_file.read())
            gni.current_waypoint_snapshots[waypoint_snapshot.id] = waypoint_snapshot
            gni.current_waypoints[waypoint.id] = waypoint
            gni.current_adj.setdefault(waypoint.id, [])
    for edge in gni.current_graph.edges:
        if len(edge.snapshot_id) == 0:
            continue
        with open(f'{gni.upload_filepath}/edge_snapshots/{edge.snapshot_id}',
                    'rb') as snapshot_file:
            edge_snapshot = map_pb2.EdgeSnapshot()
            edge_snapshot.ParseFromString(snapshot_file.read())
            gni.current_edge_snapshots[edge_snapshot.id] = edge_snapshot
            gni.current_adj[edge.id.from_waypoint].append((edge.id.to_waypoint, edge.annotations.cost.value))
    print('Uploading the graph and snapshots to the robot...')
    true_if_empty = not len(gni.current_graph.anchoring.anchors)
    response = gni.graph_nav_client.upload_graph(graph=gni.current_graph,
                                                    generate_new_anchoring=true_if_empty)
    for snapshot_id in response.unknown_waypoint_snapshot_ids:
        waypoint_snapshot = gni.current_waypoint_snapshots[snapshot_id]
        gni.graph_nav_client.upload_waypoint_snapshot(waypoint_snapshot)
        print(f'Uploaded {waypoint_snapshot.id}')
    for snapshot_id in response.unknown_edge_snapshot_ids:
        edge_snapshot = gni.current_edge_snapshots[snapshot_id]
        gni.graph_nav_client.upload_edge_snapshot(edge_snapshot)
        print(f'Uploaded {edge_snapshot.id}')
    localization_state = gni.graph_nav_client.get_localization_state()
    if not localization_state.localization.waypoint_id:
        print('\n')
        print(
            'Upload complete. Not localized.')

def navigate_to_anchor(gni:GraphNavInterface, goal):
    if not goal or len(goal) not in [2, 3, 4, 7]:
        print('Invalid arguments supplied.')
        return

    seed_T_goal = SE3Pose(float(goal[0]), float(goal[1]), 0.0, Quat())

    if len(goal) in [4, 7]:
        seed_T_goal.z = float(goal[2])
    else:
        localization_state = gni.graph_nav_client.get_localization_state()
        if not localization_state.localization.waypoint_id:
            print('Robot not localized')
            return
        seed_T_goal.z = localization_state.localization.seed_tform_body.position.z

    if len(goal) == 3:
        seed_T_goal.rot = Quat.from_yaw(float(goal[2]))
    elif len(goal) == 4:
        seed_T_goal.rot = Quat.from_yaw(float(goal[3]))
    elif len(goal) == 7:
        seed_T_goal.rot = Quat(w=float(goal[3]), x=float(goal[4]), y=float(goal[5]),
                                z=float(goal[6]))
    if not toggle_power(gni, should_power_on=True):
        print('Failed to power on the robot, and cannot complete navigate to request.')
        return
    nav_to_cmd_id = None
    is_finished = False
    while not is_finished:
        try:
            nav_to_cmd_id = gni.graph_nav_client.navigate_to_anchor(
                seed_T_goal.to_proto(), 1.0, command_id=nav_to_cmd_id)
        except ResponseError as e:
            print(f'Error while navigating {e}')
            break
        time.sleep(.5)
        is_finished = check_success(gni, nav_to_cmd_id)
    if gni.powered_on and not gni.started_powered_on:
        toggle_power(gni, should_power_on=False)

def navigate_to_gps_coords(gni:GraphNavInterface, goal):
    coords = parse_gps_goal_from_args(gni, goal)
    if not coords:
        return
    navigate_to_parsed_gps_coords(gni, coords[0], coords[1], coords[2])

def parse_gps_goal_from_args(list_of_strings:list):
    if len(list_of_strings) not in [2, 3]:
        print('Invalid arguments supplied.')
        return None
    latitude = float(list_of_strings[0])
    longitude = longitude = float(list_of_strings[1])
    yaw = None
    if len(list_of_strings) == 3:
        yaw = float(list_of_strings[2])
    return (latitude, longitude, yaw)

def navigate_to_parsed_gps_coords(gni:GraphNavInterface, latitude_degrees, longitude_degrees,
                                    yaw_around_up_radians=None):
    llh = gps_pb2.LLH(latitude=latitude_degrees, longitude=longitude_degrees, height=0.0)
    gps_params = graph_nav_pb2.GPSNavigationParams(goal_llh=llh)
    if yaw_around_up_radians:
        gps_params.goal_yaw.value = yaw_around_up_radians
    if not toggle_power(gni, should_power_on=True):
        print('Failed to power on the robot.')
        return
    nav_to_cmd_id = None
    is_finished = False
    while not is_finished:
        try:
            nav_to_cmd_id = gni.graph_nav_client.navigate_to_anchor(
                SE3Pose.from_identity().to_proto(), 1.0, command_id=nav_to_cmd_id,
                gps_navigation_params=gps_params)
        except ResponseError as e:
            print(f'Error while navigating {e}')
            break
        time.sleep(.5)
        is_finished = check_success(gni, nav_to_cmd_id)
    if gni.powered_on and not gni.started_powered_on:
        toggle_power(gni, should_power_on=False)

def navigate_to(gni:GraphNavInterface, wp_dest):
    if wp_dest is None: return
    if not toggle_power(gni, should_power_on=True):
        print('Failed to power on the robot.')
        return
    nav_to_cmd_id = None
    is_finished = False
    while not is_finished:
        try:
            nav_to_cmd_id = gni.graph_nav_client.navigate_to(wp_dest, 1.0,
                                                                command_id=nav_to_cmd_id)
        except ResponseError as e:
            print(f'Error while navigating {e}')
            break
        time.sleep(.5)
        is_finished = check_success(gni, nav_to_cmd_id)

def navigate_constrained_route(gni:GraphNavInterface, constr):
    wp_start = gni.graph_nav_client.get_localization_state().localization.waypoint_id
    if gni.current_adj is None:
        print('Current adjacency list is empty. List the waypoints and edges to update the adjacency list.')
        return
    selected = graph_nav_util.find_satisfying_waypoints(gni.current_adj, wp_start, constr)
    #print(selected)
    for wp in selected:
        time.sleep(3) # put action here
        #gni.current_metadata[wp]['metadata'] = graph_nav_util.random.randint(1,10)
        navigate_to(gni, wp)

def download_metadata(gni:GraphNavInterface):
    try:
        with open(os.path.join(gni.upload_filepath, 'waypoint_metadata.json'), 'w') as file:
            json.dump(gni.current_metadata, file, indent=4)
    except IOError as e:
        print(f'Error writing to file: {e}')
    except TypeError as e:
        print(f'Error during JSON serialization: {e}')

# will be eulerian circuit if graph is strongly connected and for each vertex in degree = out degree
def navigate_eulerian_trail(gni:GraphNavInterface, constr):
    wp_start = gni.graph_nav_client.get_localization_state().localization.waypoint_id
    if gni.current_adj is None:
        print('Current adjacency list is empty. List the waypoints and edges to update the adjacency list.')
        return
    selected = graph_nav_util.find_satisfying_waypoints(gni.current_adj, wp_start, constr)
    path = graph_nav_util.euler_circuit(gni.current_adj, wp_start)
    '''chunks=[]
    cur_chunk=[wp_start]
    ptr = 1
    for i in range(1,len(circuit)):
        cur_chunk.append(circuit[i])
        if circuit[i]==selected[ptr]:
            chunks.append(cur_chunk)
            cur_chunk=[circuit[i]]
            ptr=ptr+1 if ptr+1<len(selected) else 0
    for chunk in chunks:
        time.sleep(3) # put action here
        gni._current_metadata[chunk[0]]['metadata'] = graph_nav_util.random.randint(1,10)
        gni._navigate_to_(chunk[-1])
    ptr = 0
    for wp in circuit:
        gni._navigate_to_(wp)
        if wp==selected[ptr]:
            time.sleep(3)
            gni._current_metadata[wp]=graph_nav_util.random.randint(1,10)
            ptr+=1
            if ptr>=len(selected):
                gni._navigate_to_(circuit[-1])
                break'''
    for wp in selected:
        navigate_to(gni, wp)
        time.sleep(3)
        #gni.current_metadata[wp]['metadata']=graph_nav_util.random.randint(1,10)
    navigate_to(gni, path[-1])

def navigate_circuit_no_stop(gni:GraphNavInterface):
    wp_start = gni.graph_nav_client.get_localization_state().localization.waypoint_id
    if gni.current_adj is None:
        print('Current adjacency list is empty. List the waypoints and edges to update the adjacency list.')
        return
    circuit = graph_nav_util.euler_circuit(gni.current_adj, wp_start)
    navigate_route(circuit)

def navigate_route(gni:GraphNavInterface, waypoints):
    if not waypoints or len(waypoints) < 1:
        print('No waypoints provided for navigate route.')
        return
    edge_ids_list = []
    all_edges_found = True
    for i in range(len(waypoints) - 1):
        start_wp = waypoints[i]
        end_wp = waypoints[i + 1]
        edge_id = match_edge(gni, gni.current_edges, start_wp, end_wp)
        if edge_id is not None:
            edge_ids_list.append(edge_id)
        else:
            all_edges_found = False
            print(f'Failed to find an edge between waypoints: {start_wp} and {end_wp}')
            print(
                'List the graph\'s waypoints and edges to ensure pairs of waypoints has an edge.'
            )
            break
    if all_edges_found:
        if not toggle_power(gni, should_power_on=True):
            print('Failed to power on the robot, and cannot complete navigate route request.')
            return
        route = gni.graph_nav_client.build_route(waypoints, edge_ids_list)
        is_finished = False
        while not is_finished:
            nav_route_command_id = gni.graph_nav_client.navigate_route(
                route, cmd_duration=1.0)
            time.sleep(.5)
            is_finished = check_success(gni, nav_route_command_id)
        if gni.powered_on and not gni.started_powered_on:
            toggle_power(gni, should_power_on=False)

def clear_graph(gni:GraphNavInterface):
    return gni.graph_nav_client.clear_graph()

def toggle_power(gni:GraphNavInterface, should_power_on):
    is_powered_on = check_is_powered_on(gni)
    if not is_powered_on and should_power_on:
        power_on_motors(gni.power_client)
        motors_on = False
        while not motors_on:
            future = gni.robot_state_client.get_robot_state_async()
            state_response = future.result(
                timeout=10)
            if state_response.power_state.motor_power_state == robot_state_pb2.PowerState.STATE_ON:
                motors_on = True
            else:
                time.sleep(.25)
    elif is_powered_on and not should_power_on:
        safe_power_off_motors(gni.robot_command_client, gni.robot_state_client)
    else:
        return is_powered_on
    check_is_powered_on(gni)
    return gni.powered_on

def check_is_powered_on(gni:GraphNavInterface):
    power_state = gni.robot_state_client.get_robot_state().power_state
    gni.powered_on = (power_state.motor_power_state == power_state.STATE_ON)
    return gni.powered_on

def check_success(gni:GraphNavInterface, command_id=-1):
    if command_id == -1:
        return False
    status = gni.graph_nav_client.navigation_feedback(command_id)
    if status.status == graph_nav_pb2.NavigationFeedbackResponse.STATUS_REACHED_GOAL:
        return True
    elif status.status == graph_nav_pb2.NavigationFeedbackResponse.STATUS_LOST:
        print('Robot got lost when navigating the route, the robot will now sit down.')
        return True
    elif status.status == graph_nav_pb2.NavigationFeedbackResponse.STATUS_STUCK:
        print('Robot got stuck when navigating the route, the robot will now sit down.')
        return True
    elif status.status == graph_nav_pb2.NavigationFeedbackResponse.STATUS_ROBOT_IMPAIRED:
        print('Robot is impaired.')
        return True
    else:
        return False

def match_edge(gni:GraphNavInterface, current_edges, waypoint1, waypoint2):
    for edge_to_id in current_edges:
        for edge_from_id in current_edges[edge_to_id]:
            if (waypoint1 == edge_to_id) and (waypoint2 == edge_from_id):
                return map_pb2.Edge.Id(from_waypoint=waypoint2, to_waypoint=waypoint1)
            elif (waypoint2 == edge_to_id) and (waypoint1 == edge_from_id):
                return map_pb2.Edge.Id(from_waypoint=waypoint1, to_waypoint=waypoint2)
    return None

def get_waypoint_snapshot(gni:GraphNavInterface, id):
    if gni.current_graph is None:
        return
    for waypoint_snapshot in gni.current_waypoint_snapshots:
        if waypoint_snapshot.id == id:
            return waypoint_snapshot
    print(f'ERROR: Waypoint {id} not found in graph.')
    return None

def on_quit(gni:GraphNavInterface):
    if gni.powered_on and not gni.started_powered_on:
        gni.robot_command_client.robot_command(RobotCommandBuilder.safe_power_off_command(),
                                                    end_time_secs=time.time())