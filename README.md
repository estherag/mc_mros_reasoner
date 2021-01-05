# Reasoner

A meta-controller implementation for ROS1 applicated to UX-1 robot from UNEXMIN, based on meta-controller from [TUD](https://github.com/tud-cor/mc_mros_reasoner.git)
tested on ROS Kinetic and Ubuntu 16.04.
## Installation


#### Install java jre

The `reasoner` uses  [Owlready2](https://owlready2.readthedocs.io/en/latest/index.html) to handle the ontologies and perform reasoning. To do so, Owlready and java should be installed.

```console
sudo add-apt-repository ppa:openjdk-r/ppa
sudo apt-get update
sudo apt install openjdk-11-jdk

pip3 install owlready2
```
Under Linux, Owlready should automatically find Java.

### Create reasoner_ws and install dependencies

```console
  mkdir -p ~/reasoner_ws/src
  cd reasoner_ws/src
  git clone https://github.com/estherag/reasoner.git
  cd ..
```

### Build the code

- Once you have the workspace setup, you can build the workspace
- Do not forget to source ROS Kinetic workspace before building your `reasoner_ws`
- Specify we are using Python 3 in the building instructions as ROS Kinetic uses Python 2 by default

```console
source /opt/ros/kinetic/setup.bash
catkin build -DCATKIN_ENABLE_TESTING=0 -DCMAKE_BUILD_TYPE=Release -DPYTHON_VERSION=3.5
```

## Execution

Source your ws and launch the reasoner:

```console
source reasoner_ws/devel/setup.bash
roslaunch mros1_reasoner run.launch
```
To read diagnostic messages we use the `observer_node`. This node informs the reasoner for possible contingencies in the robot.
```console
rosrun reasoner observer_node.py
```
Two types of contingencies are managed:
- Broken thruster.

When a thruster is offline, a `/broken_thruster` topic is used to inform the observer. Just a string message with the name of the broken thruster (t0 to t7) is needed, e.g.:
```console
rostopic pub /broken_thruster std_msgs/String "data: 't0'"
```
- Change in movement direction. 

When a the movement direction chages, a `/movement` topic is used. In this experiment just 2 directions are managed: X (surge) and Z (heave). By default, the system starts with a heave movement, each time the robot changes its direction, the reasoner should be informed, e.g.:
```console
rostopic pub /movement std_msgs/String "data: 'surge'"
```
This changes in movement are implemented because for instance, when t0 thruster is broken the heave movement can be done without interfereance, as this thruster isn't implicated on this movement.

### Reconfiguration
When the reasoner detects a new configuration is required (each time a function design is grounded), a matrixAllocation msg will be published on a `/reconfiguration` topic.
This matrixAllocation msg sends one array of type float64[8] for each movement direction X, Y, Z, K, M, N.  For instance, the message sent when grounding the FD `fd_surge_all` is:

```console
X: [0.5, 0.0, -0.5, 0.0, 0.5, 0.0, -0.5, 0.0]
Y: [-0.25, -0.25, -0.25, 0.25, 0.25, -0.25, 0.25, -0.25]
Z: [0.0, -0.5, 0.0, -0.5, 0.0, 0.5, 0.0, -0.5]
K: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
M: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
N: [-0.25, 0.0, 0.25, 0.0, 0.25, 0.0, -0.25, 0.0]
``` 

This reconfiguration matrix is adapted to allocate correctly the forces according to the direction and broken thrusters, which are implemented in the ontology as function designs. This matrix values for each FD can be changed in the .csv allocated in the reconfiguration folder on the repository.

