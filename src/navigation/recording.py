import os
import time

from google.protobuf import wrappers_pb2 as wrappers

import bosdyn.client.recording
from bosdyn.api.graph_nav import map_pb2, map_processing_pb2, recording_pb2
from bosdyn.client.math_helpers import Quat, SE3Pose

import navigation.graph_nav_util as graph_nav_util
from structures.recording_client import RecordingInterface

def should_we_start_recording(ri:RecordingInterface):
    graph = ri.graph_nav_client.download_graph()
    if graph is not None:
        if len(graph.waypoints) > 0:
            localization_state = ri.graph_nav_client.get_localization_state()
            if not localization_state.localization.waypoint_id:
                return False
    return True

def clear_map(ri:RecordingInterface):
    return ri.graph_nav_client.clear_graph()

def start_recording(ri:RecordingInterface):
    should_start_recording = should_we_start_recording(ri)
    if not should_start_recording:
        print('The system is not in the proper state to start recording.'
                'Try using the graph_nav_command_line to either clear the map or'
                'attempt to localize to the map.')
        return
    try:
        status = ri.recording_client.start_recording(
            recording_environment=ri.recording_environment)
        print('Successfully started recording a map.')
    except Exception as err:
        print(f'Start recording failed: {err}')

def stop_recording(ri:RecordingInterface):
    first_iter = True
    while True:
        try:
            status = ri.recording_client.stop_recording()
            print('Successfully stopped recording a map.')
            break
        except bosdyn.client.recording.NotReadyYetError as err:
            if first_iter:
                print('Cleaning up recording...')
            first_iter = False
            time.sleep(1.0)
            continue
        except Exception as err:
            print(f'Stop recording failed: {err}')
            break

def get_recording_status(ri:RecordingInterface):
    status = ri.recording_client.get_record_status()
    if status.is_recording:
        print('The recording service is on.')
    else:
        print('The recording service is off.')

def create_default_waypoint(ri:RecordingInterface):
    resp = ri.recording_client.create_waypoint(waypoint_name='default')
    if resp.status == recording_pb2.CreateWaypointResponse.STATUS_OK:
        print('Successfully created a waypoint.')
    else:
        print('Could not create a waypoint.')

def download_full_graph(ri:RecordingInterface):
    graph = ri.graph_nav_client.download_graph()
    if graph is None:
        print('Failed to download the graph.')
        return
    write_full_graph(graph)
    print(
        f'Graph downloaded with {len(graph.waypoints)} waypoints and {len(graph.edges)} edges')
    download_and_write_waypoint_snapshots(graph.waypoints)
    download_and_write_edge_snapshots(graph.edges)

def write_full_graph(ri:RecordingInterface, graph):
    graph_bytes = graph.SerializeToString()
    write_bytes(ri.download_filepath, 'graph', graph_bytes)

def download_and_write_waypoint_snapshots(ri:RecordingInterface, waypoints):
    num_waypoint_snapshots_downloaded = 0
    for waypoint in waypoints:
        if len(waypoint.snapshot_id) == 0:
            continue
        try:
            waypoint_snapshot = ri.graph_nav_client.download_waypoint_snapshot(
                waypoint.snapshot_id)
        except Exception:
            print(f'Failed to download waypoint snapshot: {waypoint.snapshot_id}')
            continue
        write_bytes(os.path.join(ri.download_filepath, 'waypoint_snapshots'),
                            str(waypoint.snapshot_id), waypoint_snapshot.SerializeToString())
        num_waypoint_snapshots_downloaded += 1
        print(
            f'Downloaded {num_waypoint_snapshots_downloaded} of the total {len(waypoints)} waypoint snapshots.'
        )

def download_and_write_edge_snapshots(ri:RecordingInterface, edges):
    num_edge_snapshots_downloaded = 0
    num_to_download = 0
    for edge in edges:
        if len(edge.snapshot_id) == 0:
            continue
        num_to_download += 1
        try:
            edge_snapshot = ri.graph_nav_client.download_edge_snapshot(edge.snapshot_id)
        except Exception:
            # Failure in downloading edge snapshot. Continue to next snapshot.
            print(f'Failed to download edge snapshot: {edge.snapshot_id}')
            continue
        write_bytes(os.path.join(ri.download_filepath, 'edge_snapshots'),
                            str(edge.snapshot_id), edge_snapshot.SerializeToString())
        num_edge_snapshots_downloaded += 1
        print(
            f'Downloaded {num_edge_snapshots_downloaded} of the total {num_to_download} edge snapshots.'
        )

def write_bytes(ri:RecordingInterface, filepath, filename, data):
    os.makedirs(filepath, exist_ok=True)
    with open(os.path.join(filepath, filename), 'wb+') as f:
        f.write(data)
        f.close()

def update_graph_waypoint_and_edge_ids(ri:RecordingInterface, do_print=False):
    graph = ri.graph_nav_client.download_graph()
    if graph is None:
        print('Empty graph.')
        return
    ri.current_graph = graph

    localization_id = ri.graph_nav_client.get_localization_state().localization.waypoint_id

    ri.current_annotation_name_to_wp_id, ri.current_edges = graph_nav_util.update_waypoints_and_edges(
        graph, localization_id, do_print)

def list_graph_waypoint_and_edge_ids(ri:RecordingInterface):
    update_graph_waypoint_and_edge_ids(do_print=True)

def create_new_edge(ri:RecordingInterface, waypoints):
    if not waypoints or len(waypoints) != 2:
        print('Specify two waypoints (name or annotations)')
        return

    update_graph_waypoint_and_edge_ids(ri, do_print=False)

    from_id = graph_nav_util.find_unique_waypoint_id(waypoints[0], ri.current_graph,
                                                        ri.current_annotation_name_to_wp_id)
    to_id = graph_nav_util.find_unique_waypoint_id(waypoints[1], ri.current_graph,
                                                    ri.current_annotation_name_to_wp_id)

    print(f'Creating edge from {from_id} to {to_id}.')

    from_wp = get_waypoint(ri, from_id)
    if from_wp is None:
        return

    to_wp = get_waypoint(ri, to_id)
    if to_wp is None:
        return

    edge_transform = get_transform(ri, from_wp, to_wp)

    new_edge = map_pb2.Edge()
    new_edge.id.from_waypoint = from_id
    new_edge.id.to_waypoint = to_id
    new_edge.from_tform_to.CopyFrom(edge_transform)

    print(f'edge transform = {new_edge.from_tform_to}')

    ri.recording_client.create_edge(edge=new_edge)

def create_loop(ri:RecordingInterface):
    update_graph_waypoint_and_edge_ids(ri, do_print=False)

    if len(ri.current_graph.waypoints) < 2:
        print(
            f'Graph contains {len(ri.current_graph.waypoints)} waypoints -- at least two are '
            f'needed to create loop.')
        return False

    sorted_waypoints = graph_nav_util.sort_waypoints_chrono(ri.current_graph)
    edge_waypoints = [sorted_waypoints[-1][0], sorted_waypoints[0][0]]

    create_new_edge(ri, edge_waypoints)

def auto_close_loops_prompt(ri:RecordingInterface):
    print("""
    Options:
    (0) Close all loops.
    (1) Close only fiducial-based loops.
    (2) Close only odometry-based loops.
    (q) Back.
    """)
    try:
        inputs = input('>')
    except NameError:
        return
    req_type = str.split(inputs)[0]
    close_fiducial_loops = False
    close_odometry_loops = False
    if req_type == '0':
        close_fiducial_loops = True
        close_odometry_loops = True
    elif req_type == '1':
        close_fiducial_loops = True
    elif req_type == '2':
        close_odometry_loops = True
    elif req_type == 'q':
        return
    else:
        print('Unrecognized command. Going back.')
        return
    auto_close_loops(ri, close_fiducial_loops, close_odometry_loops)

def auto_close_loops(ri:RecordingInterface, close_fiducial_loops, close_odometry_loops):
    response = ri.map_processing_client.process_topology(
        params=map_processing_pb2.ProcessTopologyRequest.Params(
            do_fiducial_loop_closure=wrappers.BoolValue(value=close_fiducial_loops),
            do_odometry_loop_closure=wrappers.BoolValue(value=close_odometry_loops)),
        modify_map_on_server=True)
    print(f'Created {len(response.new_subgraph.edges)} new edge(s).')

def optimize_anchoring(ri:RecordingInterface):
    response = ri.map_processing_client.process_anchoring(
        params=map_processing_pb2.ProcessAnchoringRequest.Params(),
        modify_anchoring_on_server=True, stream_intermediate_results=False,
        apply_gps_results=ri.use_gps)
    if response.status == map_processing_pb2.ProcessAnchoringResponse.STATUS_OK:
        print(f'Optimized anchoring after {response.iteration} iteration(s).')
        if ri.use_gps:
            print('Downloading updated graph...')
            ri.current_graph = ri.graph_nav_client.download_graph()
    else:
        print(f'Error optimizing {response}')

def get_waypoint(ri:RecordingInterface, id):
    if ri.current_graph is None:
        ri.current_graph = ri.graph_nav_client.download_graph()
    for waypoint in ri.current_graph.waypoints:
        if waypoint.id == id:
            return waypoint
    print(f'ERROR: Waypoint {id} not found in graph.')
    return None

def get_transform(ri:RecordingInterface, from_wp, to_wp):
    from_se3 = from_wp.waypoint_tform_ko
    from_tf = SE3Pose(
        from_se3.position.x, from_se3.position.y, from_se3.position.z,
        Quat(w=from_se3.rotation.w, x=from_se3.rotation.x, y=from_se3.rotation.y,
                z=from_se3.rotation.z))
    to_se3 = to_wp.waypoint_tform_ko
    to_tf = SE3Pose(
        to_se3.position.x, to_se3.position.y, to_se3.position.z,
        Quat(w=to_se3.rotation.w, x=to_se3.rotation.x, y=to_se3.rotation.y,
                z=to_se3.rotation.z))
    from_T_to = from_tf.mult(to_tf.inverse())
    return from_T_to.to_proto()