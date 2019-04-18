import argparse
import time
import msgpack
from enum import Enum, auto

import numpy as np
import random

from utils.planning_utils import a_star, heuristic, create_grid
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local, local_to_global


class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()


class MotionPlanning(Drone):

    def __init__(self, connection):
        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 1.0:
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.1:
                if abs(self.local_position[2]) < 0.01:
                    self.disarming_transition()

    def state_callback(self):
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.plan_path()
            elif self.flight_state == States.PLANNING:
                self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.manual_transition()

    def arming_transition(self):
        self.flight_state = States.ARMING
        print("arming transition")
        self.arm()
        self.take_control()

    def takeoff_transition(self):
        self.flight_state = States.TAKEOFF
        print("takeoff transition")
        self.takeoff(self.target_position[2])

    def waypoint_transition(self):
        self.flight_state = States.WAYPOINT
        print("waypoint transition")
        self.target_position = self.waypoints.pop(0)
        print('target position', self.target_position)
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], self.target_position[3])

    def landing_transition(self):
        self.flight_state = States.LANDING
        print("landing transition")
        self.land()

    def disarming_transition(self):
        self.flight_state = States.DISARMING
        print("disarm transition")
        self.disarm()
        self.release_control()

    def manual_transition(self):
        self.flight_state = States.MANUAL
        print("manual transition")
        self.stop()
        self.in_mission = False

    def send_waypoints(self):
        print("Sending waypoints to simulator ...")
        data = msgpack.dumps(self.waypoints)
        self.connection._master.write(data)

    def plan_path(self):
        self.flight_state = States.PLANNING
        print("Searching for a path ...")
        TARGET_ALTITUDE = 5
        SAFETY_DISTANCE = 5

        self.target_position[2] = TARGET_ALTITUDE


        def converter(s):
            l = str(s, 'utf-8').split(' ')
            d ={l[0]:float(l[1])}

            return d

        # Read lat0, lon0 from colliders into floating point values
        data = np.genfromtxt('map/colliders.csv', delimiter=',', dtype=object, max_rows=1, autostrip=True, converters={0:converter, 1:converter})

        # Add data points to a global home position dictionary
        global_home_pos = dict()
        for d in data:
            global_home_pos.update(d)

        # Set home position to (lon0, lat0, 0)
        self.set_home_position(global_home_pos['lon0'],
                               global_home_pos['lat0'],
                               0.0)
        print("Home Position Set: [", global_home_pos['lon0'], ", ",  global_home_pos['lat0'], "]")


        # Retrieve current global position
        curr_global_pos = self.global_position

        # Debugging
        #print("Current Global Position: ", curr_global_pos)


        # Convert to current local position using global_to_local()
        curr_local_pos = global_to_local(curr_global_pos, self.global_home)

        # Debugging
        #print("Current Local Position: ", curr_local_pos)

        # Read in obstacle map
        data = np.loadtxt('map/colliders.csv', delimiter=',', dtype='Float64', skiprows=2)

        # Define a grid for a particular altitude and safety margin around obstacles
        grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
        print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))

        # Define starting point on the grid (this is just grid center)
        #grid_start = (-north_offset, -east_offset)

        # Define starting point on the grid based on current position rather than map center
        grid_start = (int(curr_local_pos[0])-north_offset, int(curr_local_pos[1])-east_offset)

        # Set goal as some arbitrary position on the grid
        #grid_goal = (-north_offset + 10, -east_offset + 10)

        # Goal position can be selected either using local or global coordinates
        # In this example, use global (lat, lon) coordinates to select goal location
        local_goal = False

        if local_goal:
            # Set goal as position on the grid relative to the current position
            grid_goal = (int(curr_local_pos[0])-north_offset + 10, int(curr_local_pos[1])-east_offset + 10)
        else:
            # Sets goal as latitude / longitude position and convert
            # Convert current local position to global coordinates
            goal_coord = local_to_global([curr_local_pos[0], curr_local_pos[1], curr_local_pos[2]], self.global_home)

            # Add latitude and longitude values to the goal location
            goal_coord = (goal_coord[0] + 0.002, goal_coord[1], goal_coord[2] + 60)

            # Convert back to local coordinates to send to the drone
            goal_pos = global_to_local(goal_coord, self.global_home)
            grid_goal = (int(goal_pos[0])-north_offset, int(goal_pos[1])-east_offset)


        #Check that goal is within bounds and does not land on a restricted zone
        if grid_goal[0] > 921:
            print("ERROR: Goal Coordinate 0 - ", grid_goal[0], " is Out of Bounds!")
            self.landing_transition()
            return
        if grid_goal[1] > 921:
            print("ERROR: Goal Coordinate 1 - ", grid_goal[1], " is Out of Bounds!")
            self.landing_transition()
            return
        if grid[grid_goal[0], grid_goal[1]] == 1:
            print("ERROR: Goal Coordinate is not flyable!")
            self.landing_transition()
            return

        # Debugging
        #print("Goal Position: ", goal_pos)
        print('Local Start and Goal: ', grid_start, grid_goal)

        # Run A* to find a path from start to goal
        # Added diagonal motions with a cost of sqrt(2) to A* implementation
        print('Searching for Path...')
        path, _ = a_star(grid, heuristic, grid_start, grid_goal)


        # Adds z coordinate to waypoint
        def point(p):
            return np.array([p[0], p[1], 1.])

        # Check if floating point values are collinear
        def collinear(p1, p2, p3, epsilon=1e-6):
            collinear = False

            # Add points as rows in a matrix
            mat = np.vstack((point(p1), point(p2), point(p3)))
            # Calculate determinant of the matrix
            det = np.linalg.det(mat)

            # Collinear is true if the determinant is less than epsilon
            if det < epsilon:
                collinear = True

            return collinear

        # Prune path to minimize number of waypoints
        i = 0
        # Loop through all waypoints in groups of three
        while i < len(path) - 2:
            p1 = path[i]
            p2 = path[i+1]
            p3 = path[i+2]

            # If the points are collinear, remove the middle waypoint
            if collinear(p1, p2, p3):
                path.remove(path[i+1])
            else:
                i += 1


        # Convert path to waypoints
        waypoints = [[p[0] + north_offset, p[1] + east_offset, TARGET_ALTITUDE, 0] for p in path]

        # Set self.waypoints
        self.waypoints = waypoints

        #send waypoints to sim (this is just for visualization of waypoints)
        self.send_waypoints()

    def start(self):
        self.start_log("Logs", "NavLog.txt")

        print("starting connection")
        self.connection.start()

        # Only required if they do threaded
        # while self.in_mission:
        #    pass

        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=60)
    drone = MotionPlanning(conn)
    time.sleep(1)

    drone.start()
