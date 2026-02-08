# Multi-Robot Systems: Swarm Control using Consensus Protocols and Reynolds' Rules

This project implements consensus protocols for multi-robot systems in ROS2, building upon previous work using Reynolds' rules for collision avoidance. It utilizes the Crazyflies simulation in the CrazySim Gazebo environment.

| Reynolds' Rules (3 drones) | Reynolds' Rules (4 drones) | Consensus Formation |
|:---:|:---:|:---:|
| ![Reynolds Rules](images/Reynolds_Rules.gif) | ![Reynolds Rules 4 Drones](images/Reynolds_rules_4drones.gif) | ![Consensus Formation](images/Consensus.gif) |
| Rendezvous using flocking behaviors: separation, cohesion, alignment, and migration | Same flocking rendezvous scaled to 4 drones | Formation control using consensus protocol with leader-follower migration |

## Project Structure

```
mrs_project_crazyflies/
├── mrs_project_crazyflies/        # Main Python package
│   ├── reynolds.py                # Reynolds flocking algorithm controller
│   ├── consensus_rendezvous.py    # Consensus-based rendezvous protocol
│   ├── consensus_formation.py     # Consensus-based formation control
│   └── svc.py                     # State validity checker for obstacle avoidance
├── launch/
│   └── reynolds.launch.py         # Launch file for spawning multiple Reynolds boids
├── test/                          # ROS2 linter tests (flake8, pep257, copyright)
├── package.xml                    # ROS2 package metadata
├── CMakeLists.txt                 # Build configuration
├── setup.py                       # Python package installation & entry points
└── setup.cfg                      # Install directory config
```

## Architecture Overview

The system provides three control algorithms that can be run independently. All algorithms operate on Crazyflie drones simulated in Gazebo via CrazySim. Robots exchange state information through ROS2 odometry topics (`/cf_i/odom`) and receive velocity commands on `/cf_i/cmd_vel`.

```
                    CrazySim (Gazebo)
                          │
           ┌──────────────┼──────────────┐
           │              │              │
         cf_1           cf_2          cf_3/cf_4
           │              │              │
           └──── Odometry Exchange ──────┘
                          │
              ┌───────────┼───────────┐
              │           │           │
         Reynolds     Consensus   Consensus
         Flocking    Rendezvous   Formation
              │           │           │
              └───────────┼───────────┘
                          │
                     cmd_vel → Drones
```

## Control Algorithms

### 1. Reynolds Flocking (`reynolds.py`)

Each robot runs an independent `BoidController` node that implements five weighted behaviors:

| Behavior | Weight | Description |
|----------|--------|-------------|
| **Separation** | 0.3 | Repels from nearby neighbors using inverse-distance weighting |
| **Cohesion** | 0.7 | Steers toward the center of mass of visible neighbors |
| **Alignment** | 0.2 | Matches velocity with neighbors |
| **Migration** | 0.7 | Steers toward a goal position published on `/goal_pose` |
| **Obstacle avoidance** | 0.8 | Repels from occupied cells in the occupancy grid map |

The main control loop runs at 10 Hz. At each step, the five acceleration vectors are computed, combined as a weighted sum, and clamped to a maximum speed of 0.5 m/s. Neighbors are only considered if they are within 1 m distance and within a pi-radian field-of-view cone.

**Consensus integration:** When a consensus velocity is published on `/cf_i/consensus_vel`, the Reynolds controller switches to a hybrid mode that combines separation with the consensus velocity, allowing the consensus algorithms to override flocking behavior.

### 2. Consensus Rendezvous (`consensus_rendezvous.py`)

Drives all robots to converge at a common meeting point using distributed consensus on a communication graph.

**Algorithm:**

Given robot positions **X** and an adjacency matrix **A** defining which robots can communicate:

1. Compute the degree matrix **D** (diagonal matrix of row sums of **A**)
2. Compute the graph Laplacian **L = D - A**
3. Compute velocity: **V = -L * X * dt**

This drives each robot toward the weighted average of its neighbors' positions. Under a connected topology, the swarm converges to the centroid of the initial positions.

**Supported topologies (3 robots):**

| Topology | Graph | Adjacency |
|----------|-------|-----------|
| 1 - Ring | 1 &rarr; 2 &rarr; 3 &rarr; 1 | Directed cycle |
| 2 - Fully connected | All pairs | Complete graph |

### 3. Consensus Formation (`consensus_formation.py`)

Extends the rendezvous protocol to maintain geometric formations while navigating toward a goal.

**Algorithm:**

Given desired formation offsets **&xi;**, current positions **X**, and Laplacian **L**:

1. Compute formation error: **e = X - &xi;**
2. Compute velocity: **V = -L * e * dt**
3. The leader (robot closest to the goal) additionally receives a migration velocity toward `/goal_pose`

**Available formations (4 robots):**

| Formation | Offsets |
|-----------|---------|
| `square` | (0,1), (1,1), (1,0), (0,0) |
| `triangle` | (0,0), (1,1), (1,2), (2,0) |
| `line` | (0,0), (1,0), (2,0), (3,0) |

**Available formations (3 robots):**

| Formation | Offsets |
|-----------|---------|
| `triangle` | (0,0), (1,2), (2,0) |
| `line_h` | (0,0), (1,0), (2,0) |
| `line_v` | (0,0), (0,1), (0,2) |

The leader robot is dynamically selected as the one closest to the current goal position.

### 4. State Validity Checker (`svc.py`)

Utility class used by the Reynolds controller for obstacle avoidance. It subscribes to an `OccupancyGrid` map from the map server, converts world coordinates to grid indices, and checks whether a given position is collision-free. Cells with an occupancy value >= 5 are treated as obstacles. A robot footprint check verifies clearance in the neighborhood around a target cell.

## ROS2 Topics

| Topic | Type | Used by | Direction |
|-------|------|---------|-----------|
| `/cf_i/odom` | Odometry | All algorithms | Subscribe |
| `/cf_i/cmd_vel` | Twist | All algorithms | Publish |
| `/goal_pose` | PoseStamped | Reynolds, Formation | Subscribe |
| `/cf_i/consensus_vel` | Twist | Reynolds | Subscribe |
| `/cf_i/velocity_marker` | MarkerArray | Reynolds | Publish |
| `/cf_i/path` | Path | Reynolds | Publish |
| `/map_server/map` | GetMap (service) | Reynolds | Client |

## How to Run

**Prerequisites:**

* ROS2 installed and configured
* CrazySim ROS2 workspace cloned and built (`colcon build --packages-select mrs_project_crazyflies --merge-install`)
* Workspace sourced (`source install/setup.sh`)

**Reynolds Flocking:**

```bash
ros2 launch mrs_project_crazyflies reynolds.launch.py
```

The launch file spawns one `BoidController` node per robot. The number of robots defaults to 4 and can be set via the `NUM_ROBOTS` environment variable.

**Consensus Rendezvous:**

```bash
ros2 run mrs_project_crazyflies consensus_rendezvous --ros-args -p topology:=1
```

The `topology` parameter selects the communication graph (1 = ring, 2 = fully connected).

**Consensus Formation:**

```bash
ros2 launch mrs_project_crazyflies reynolds.launch.py
ros2 run mrs_project_crazyflies consensus_formation --ros-args -p topology:=1 -p formation:="square"
```

The Reynolds launch must be running first so that each drone has a boid controller handling separation and velocity commands. The consensus formation node then publishes consensus velocities that override the flocking behavior.
