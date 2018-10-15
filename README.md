Robot Kinematic Services [![Build Status](https://travis-ci.org/diogoalmeida/robot_kinematic_services.svg?branch=master)](https://travis-ci.org/diogoalmeida/robot_kinematic_services)
===
This package implements ROS services for computing the forward and inverse kinematics (resp. FK and IK) for a robot kinematic chain. It loads your robot's URDF description and allows you to compute
the kinematics for a given chain end-effector or any point rigidly attached to it (e.g., a tooltip).

### Usage
This package assumes that the robot kinematic description is available at the `/robot_description` rosparameter. The fk and ik services are available, respectively, as `compute_fk` and `compute_ik`. These values can be remaped in the launch file.

A single parameter is used, `robot_chain_base_link`, which should be set for the base link name of your robot kinematic chain. A minimal launch file is provided in this package, where this parameter is set to the base like of the fetch manipulator, `torso_lift_link`:

```xml
<launch>
  <node name="kinematic_services" type="kinematic_services" pkg="robot_kinematic_services" output="screen">
    <rosparam>
      robot_chain_base_link: torso_lift_link
    </rosparam>
  </node>
</launch>
```

If you want different names for the provided services, that can be achieved as in the following example.

```xml
<launch>
  <node name="kinematic_services" type="kinematic_services" pkg="robot_kinematic_services" output="screen">
    <remap from="compute_fk" to="forward_kinematics"/>
    <remap from="compute_ik" to="inverse_kinematics"/>
    <rosparam>
      robot_chain_base_link: torso_lift_link
    </rosparam>
  </node>
</launch>
```

Both service requests include the parameters `chain_end_effector_name` and `tooltip_name`. These should be set, respectively, to the last link of the robot kinematic chain, and the name of the frame with respect to you want to compute the solution. That is, forward kinematics are computed for the tooltip and inverse kinematic solutions assume that the given pose is of the tooltip. **The tooltip frame should be rigidly attached to the end-effector** for solutions to be valid.

Example: the chain end-effector of the fetch robot is `gripper_link`. If you want to compute the inverse kinematics for that point you can call the `compute_ik` service as

```
$ rosservice call /compute_ik "chain_end_effector_name: 'gripper_link'
desired_pose:
  header:
    seq: 0
    stamp:
      secs: 0
      nsecs: 0
    frame_id: 'torso_lift_link'
  pose:
    position:
      x: 0.282
      y: -0.673
      z: 0.013
    orientation:
      x: -0.195
      y: 0.673
      z: 0.121
      w: 0.704"
```
and a possible answer will be
```
status:
  code: 0
ik_solution: [-1.6031114799568484, -0.3187570069269499, -2.5669119402583984, -1.2030817908523226, 2.403965512561466, 0.7742537749351657, -1.3350190150363965]
```

Note that the pose can be expressed with respect to any desired frame, not necessarily the `robot_chain_base_link`.

### Installation
This packages uses the `KDL_manager` of the [generic_control_toolbox](https://github.com/diogoalmeida/generic_control_toolbox) to compute the kinematic solutions. You can use the `trac-ik` branch of that package to improve the quality of the IK results. It has been tested in ROS Indigo, but should work in recent distros as well.

Example workspace configuration:
```
cd ~
mkdir -p catkin_ws/src
cd catkin_ws/src
catkin_init_workspace
git clone https://github.com/diogoalmeida/generic_control_toolbox.git
git clone https://github.com/diogoalmeida/robot_kinematic_services.git
cd ..
catkin_make
```

To setup track-ik, you should install it:
```
sudo apt-get install ros-<distro>-trac-ik
```
where `<distro>` can be `indigo`, `kinetic` or `jade`, and should match your ROS distribution. Then, you should checkout the appropriate branch of the control toolbox:

```
cd ~/catkin_ws/src/generic_control_toolbox
git checkout trac-ik
cd ~/catkin_ws
catkin_make
```
