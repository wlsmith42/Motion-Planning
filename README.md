# 3D Motion Planning

---

### Udacity Flying Car and Autonomous Flight Engineer Nanodegree

![Quad Image](./images/enroute.png)
---

The goals / steps of this project are the following:

* Load the 2.5D map in the colliders.csv file describing the environment.
* Discretize the environment into a grid or graph representation.
* Define the start and goal locations based on the car's current location.
* Perform a search using A* or other search algorithm.
* Use a collinearity test or ray tracing method (like Bresenham) to remove unnecessary waypoints.
* Return waypoints in local ECEF coordinates (format for `self.all_waypoints` is [N, E, altitude, heading], where the drone’s start location corresponds to [0, 0, 0, 0].

---





This project is a continuation of the [Backyard Flyer project](https://github.com/wlsmith42/Backyard-Flyer) where I executed a simple square shaped flight path. In this project I integrated the techniques that I learned throughout the last several lessons to plan a path through an urban environment. 


## To complete this project on your local machine, follow these instructions:
### Step 1: Download the Simulator
This is a new simulator environment!  

Download the Motion-Planning simulator for this project that's appropriate for your operating system from the [simulator releases repository](https://github.com/udacity/FCND-Simulator-Releases/releases).

### Step 2: Set up your Python Environment
If you haven't already, set up your Python environment and get all the relevant packages installed using Anaconda following instructions in [this repository](https://github.com/udacity/FCND-Term1-Starter-Kit)

### Step 3: Clone this Repository
```sh
git clone https://github.com/wlsmith42/Motion-Planning
```

### Step 4: Test setup
The first task in this project is to test the [solution code](https://github.com/wlsmith42/Motion-Planning/blob/master/backyard_flyer_solution.py) for the Backyard Flyer project in this new simulator. Verify that your Backyard Flyer solution code works as expected and your drone can perform the square flight path in the new simulator. To do this, start the simulator and run the `backyard_flyer_solution.py` script.

```sh
source activate fcnd # if you haven't already sourced your Python environment, do so now.
python backyard_flyer_solution.py
```
The quad should take off, fly a square pattern and land, just as in the previous project. If everything functions as expected then you are ready to start work on this project.

## Rubric Points

### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it! Below I describe how I addressed each rubric point and where in my code each point is handled.

### Explain the Starter Code

#### 1. Explain the functionality of what's provided in `motion_planning.py` and `planning_utils.py`

In addition to the event-driven flight states from the backyard flyer project, `motion_planning.py` contains the code to read in an obstacle map and construct a corresponding grid with a safety margin around obstacles as shown in the code below:

```Python
# Read in obstacle map
data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)
        
# Define a grid for a particular altitude and safety margin around obstacles
grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))

# Define starting point on the grid (this is just grid center)
grid_start = (-north_offset, -east_offset)
```

The file `planning_utils.py` included a basic implementation of the A* Search algorithm along with its heuristic function and valid action states along with a function that creates a grid based on obstacle data, drone altitude, and safety disnace (which was used in the code block above).

### Implementing Your Path Planning Algorithm

#### 1. Set your global home position

I read in the first line of the csv file using numpy's `genfromtxt` function, I then simply iterated over the data and added it to a global home position dictionary. Then I used the lat0 and lon0 floating point values along with the `set_home_position` function to update the vehicle's global home position. The code to accomplish this is shown below:


```Python
data = np.genfromtxt('map/colliders.csv', delimiter=',', dtype=object, max_rows=1, autostrip=True, converters={0:converter, 1:converter})

global_home_pos = dict()
for d in data:
global_home_pos.update(d)

# Set home position to (lon0, lat0, 0)
self.set_home_position(global_home_pos['lon0']
                       global_home_pos['lat0'],
                       0.0)
print("Home Position Set: [", global_home_pos['lon0'], ", ",  global_home_pos['lat0'], "]")
```

#### 2. Set your current local position

There is a function in the udacidrone library to easily convert local coordinates to global coordinates and vice versa. Here I used the `global_to_local` function in order to convert the global coordinates read in from the csv file to a local coordinate system.

```Python
# Convert to current local position using global_to_local()
curr_local_pos = global_to_local(curr_global_pos, self.global_home)
```

#### 3. Set grid start position from local position
This is another step in adding flexibility to the start location. This allows the vehicle to start its path from its current location, and not always map center. To accomplish this, I set the grid_start to the current local position minus any offset for that axis. The code can be seen below:

```Python
# Define starting point on the grid based on current position rather than map center
grid_start = (int(curr_local_pos[0])-north_offset, int(curr_local_pos[1])-east_offset)
```

#### 4. Set grid goal position from geodetic coords
This step is to add flexibility to the desired goal location. This allows any (lat, lon) within the map to be chosen as the goal location and have it rendered to a location on the grid. The first step is to convert the existing coordinates to global coordinate space before adding the desired (lat, lon) values. Once the global goal position has been updated, the coordinated need to be converted back to local coordinates to send to the next steps in the path planner. The code is shown below:

```Python
# Adapt to set goal as latitude / longitude position and convert
# Convert current local position to global coordinates
goal_coord = local_to_global([curr_local_pos[0], curr_local_pos[1], curr_local_pos[2]], self.global_home)

# Add latitude and longitude values to the goal location
goal_coord = (goal_coord[0] + 0.002, goal_coord[1], goal_coord[2] + 60)

# Convert back to local coordinates to send to the drone
goal_pos = global_to_local(goal_coord, self.global_home)
grid_goal = (int(goal_pos[0])-north_offset, int(goal_pos[1])-east_offset)
```

#### 5. Modify A* to include diagonal motion (or replace A* altogether)

To include diagonal motion in the A* Search I added two parts to the `planning_utils` file. The first was four addition actions that could move the vehicle diagonally along the grid with a cost of sqrt(2).

```Python
# diagonal directions
NORTH_WEST = (-1, -1, np.sqrt(2))
NORTH_EAST = (-1, 1, np.sqrt(2))
SOUTH_WEST = (1, -1, np.sqrt(2))
SOUTH_EAST = (1, 1, np.sqrt(2))
```

Secondly, I included validation checks that would remove diagonal actions from the valid action list if the vehicle was too close to the boundary of the map or an obstacle

```Python
if x - 1 < 0 or y - 1 < 0 or grid[x - 1, y - 1] == 1:
	valid_actions.remove(Action.NORTH_WEST)
if x - 1 < 0 or y + 1 > m or grid[x - 1, y + 1] == 1:
	valid_actions.remove(Action.NORTH_EAST)
if x + 1 > n or y - 1 < 0 or grid[x + 1, y - 1] == 1:
	valid_actions.remove(Action.SOUTH_WEST)
if x + 1 > n or y + 1 > m or grid[x + 1, y - 1] == 1:
	valid_actions.remove(Action.SOUTH_EAST)
```

#### 6. Cull waypoints

The first step to cull waypoints was to create a collinearity test that would return true if the three given points were in a line. The collinear function is shown below:

```Python
def collinear(p1, p2, p3):
	collinear = False

	# Add points as rows in a matrix
	mat = np.vstack((point(p1), point(p2), point(p3)))
	
	# Calculate determinant of the matrix
	det = np.linalg.det(mat)

	# Collinear is true if the determinant is less than epsilon
	if det < epsilon:
		collinear = True

	return collinear

```
The next code block shows path pruning using the collinearity test. If the three waypoints are a straight line, the middle waypoint is pruned from the list.

```Python
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
```

In the future I would like to test ray tracking methods like Bresenham's algorithm as a replacement to the collinearity test.

### Execute the flight
#### 1. Does it work?

It works! Given a goal position, the drone is successfully able to plan a path using A* Search, prune the path using a collinearity test, and execute the flight plan while avoiding obstacles. The results can be seen in the gif below:

![Flight Path](./images/flight_path.gif)



## Future Work 
The submission covers all requirements for this project that are laid out in the rubric, but these items can be added later on to take this project above and beyond.

### Real World Planning
For an extra challenge, consider implementing some of the techniques described in the "Real World Planning" lesson. You could try implementing a vehicle model to take dynamic constraints into account, or implement a replanning method to invoke if you get off course or encounter unexpected obstacles.

### Try flying more complex trajectories
In this project, things are set up nicely to fly right-angled trajectories, where you ascend to a particular altitude, fly a path at that fixed altitude, then land vertically. However, you have the capability to send 3D waypoints and in principle you could fly any trajectory you like. Rather than simply setting a target altitude, try sending altitude with each waypoint and set your goal location on top of a building!

### Adjust your deadbands
Adjust the size of the deadbands around your waypoints, and even try making deadbands a function of velocity. To do this, you can simply modify the logic in the `local_position_callback()` function.

### Add heading commands to your waypoints
This is a recent update! Make sure you have the [latest version of the simulator](https://github.com/udacity/FCND-Simulator-Releases/releases). In the default setup, you're sending waypoints made up of NED position and heading with heading set to 0 in the default setup. Try passing a unique heading with each waypoint. If, for example, you want to send a heading to point to the next waypoint, it might look like this:

```python
# Define two waypoints with heading = 0 for both
wp1 = [n1, e1, a1, 0]
wp2 = [n2, e2, a2, 0]
# Set heading of wp2 based on relative position to wp1
wp2[3] = np.arctan2((wp2[1]-wp1[1]), (wp2[0]-wp1[0]))
```

This may not be completely intuitive, but this will yield a yaw angle that is positive counterclockwise about a z-axis (down) axis that points downward.

Put all of these together and make up your own crazy paths to fly! Can you fly a double helix??
![Double Helix](./images/double_helix.gif)

Ok flying a double helix might seem like a silly idea, but imagine you are an autonomous first responder vehicle. You need to first fly to a particular building or location, then fly a reconnaissance pattern to survey the scene! Give it a try!
