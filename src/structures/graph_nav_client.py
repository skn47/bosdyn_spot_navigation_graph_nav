from bosdyn.client.graph_nav import GraphNavClient
from bosdyn.client.power import PowerClient
from bosdyn.client.robot_command import RobotCommandClient
from bosdyn.client.robot_state import RobotStateClient

class GraphNavInterface(object):
    def __init__(self, robot, upload_path, use_gps=False):
        self.robot = robot
        self.use_gps = use_gps

        self.robot.time_sync.wait_for_sync()

        self.robot_command_client = self.robot.ensure_client(RobotCommandClient.default_service_name)
        self.robot_state_client = self.robot.ensure_client(RobotStateClient.default_service_name)

        self.graph_nav_client = self.robot.ensure_client(GraphNavClient.default_service_name)

        self.power_client = self.robot.ensure_client(PowerClient.default_service_name)

        power_state = self.robot_state_client.get_robot_state().power_state
        self.started_powered_on = (power_state.motor_power_state == power_state.STATE_ON)
        self.powered_on = self.started_powered_on

        self.max_attempts_to_wait = 50

        self.current_graph = None
        self.current_adj = {}
        self.current_waypoints = dict()  # maps id to waypoint
        self.current_edges = dict()  # maps to_waypoint to list(from_waypoint)
        self.current_waypoint_snapshots = dict()  # maps id to waypoint snapshot
        self.current_edge_snapshots = dict()  # maps id to edge snapshot
        self.current_annotation_name_to_wp_id = dict()

        self.current_metadata = {}

        if upload_path[-1] == '/':
            self.upload_filepath = upload_path[:-1]
        else:
            self.upload_filepath = upload_path