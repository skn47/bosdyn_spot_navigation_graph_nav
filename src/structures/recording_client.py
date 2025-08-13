import os

from bosdyn.client.graph_nav import GraphNavClient
from bosdyn.client.map_processing import MapProcessingServiceClient
from bosdyn.client.recording import GraphNavRecordingServiceClient


class RecordingInterface(object):
    def __init__(self, robot, download_filepath, download_name, client_metadata, use_gps=False):
        self.robot = robot
        self.use_gps = use_gps
        
        self.robot.time_sync.wait_for_sync()

        self.download_filepath = os.path.join(download_filepath, download_name)

        self.recording_client = self.robot.ensure_client(
            GraphNavRecordingServiceClient.default_service_name)

        self.recording_environment = GraphNavRecordingServiceClient.make_recording_environment(
            waypoint_env=GraphNavRecordingServiceClient.make_waypoint_environment(
                client_metadata=client_metadata))

        self.graph_nav_client = robot.ensure_client(GraphNavClient.default_service_name)

        self.map_processing_client = robot.ensure_client(
            MapProcessingServiceClient.default_service_name)

        self.current_graph = None
        self.current_edges = dict()  #maps to_waypoint to list(from_waypoint)
        self.current_waypoint_snapshots = dict()  # maps id to waypoint snapshot
        self.current_edge_snapshots = dict()  # maps id to edge snapshot
        self.current_annotation_name_to_wp_id = dict()