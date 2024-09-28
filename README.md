# Secure-Formation-Control-For-UAV-Swarm

## exp_uavs_collision_avoidance

#### Introduction

An experiment for UAVs swarm secure controller, transfering control commands through mavros topics. 

#### Installation

```
git clone https://gitee.com/bhswift/exp_uavs_collision_avoidance.git
cd exp_collision_avoidance/
catkin_make
source ./devel/setup.bash
```

#### Usage

1.  Starting minigc, connecting to UAVs 1 to 4 and arming manually.
2.  Running the algorithm expriment ROS node.

```
rosrun exp_avoid_collision exp_avoid_collision_node
```

3.  Sending commands to ROS topic /command.  
    0 represent Stay (default)  
    1 represent Takeoff  
    2 represent Running Algorithm  
    8 represent Next Destination
    9 represent Landing  

## exp_uavs_dos

#### Introduction

An experiment for UAVs swarm secure controller, transfering control commands through mavros topics. 

#### Installation

```
git clone https://gitee.com/bhswift/exp_uavs_dos.git
cd exp_uavs_dos/
catkin_make
source ./devel/setup.bash
```

#### Usage

1.  Starting minigc, connecting to UAVs 1 to 4 and arming manually.
2.  Running the algorithm expriment ROS node.

```
rosrun exp_algorithm exp_algorithm_node
```

3.  Sending commands to ROS topic /command.  
    0 represent Stay (default)  
    1 represent Takeoff  
    2 represent Running Algorithm  
    9 represent Landing  

## simulation_uavs_collision_avoidance

#### Introduction

The MATLAB simulation code for time-varying formation tracking control with collision_avoidance, including calculator, solver and static/dynamic plotter.

#### Installation

```
git clone https://gitee.com/bhswift/simulation_uavs_collision_avoidance.git
```

#### Usage

1.  Design a the parameter of the algorithm by running parameter_solver.m 
2.  Run main.m 
3.  Run plotter.m to plot the data

## simulation_uavs_dos

#### Introduction

The MATLAB simulation code for time-varying formation tracking control under DoS attack, including calculator, solver and static/dynamic plotter.

#### Installation

```
git clone https://gitee.com/bhswift/simulation_uavs_dos.git
```

#### Usage

1.  Design a the parameter of the algorithm by running parameter_solver.m 
2.  Run main.m 
3.  Run plotter.m to plot the data