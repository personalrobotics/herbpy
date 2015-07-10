HERBPy
======

HerbPy is a Python library for interacting with HERB through OpenRAVE. HERB is
a bimanual mobile manipulator designed and built by the [Personal Robotics Lab]
(https://personalrobotics.ri.cmu.edu) at [Carnegie Mellon University]
(http://www.cmu.edu). HerbPy expands the robot-agnostic helper library [PrPy]
(https://github.com/personalrobotics/prpy) by wrapping HERB-specific functionality.

## Installation ##
The following rosinstall can be used to get the minimum dependencies required
to use HerbPy in a catkin workspace:
```
- git: {local-name: herbpy, uri: 'git@github.com:personalrobotics/herbpy'}
- git: {local-name: prpy, uri: 'git@github.com:personalrobotics/prpy'}
- git: {local-name: herb_description, uri: 'git@github.com:personalrobotics/herb_description'}
- git: {local-name: openrave_catkin, uri: 'git@github.com:personalrobotics/openrave_catkin'}
- git: {local-name: or_parabolicsmoother, uri: 'git@github.com:personalrobotics/or_parabolicsmoother'}
- git: {local-name: or_trajopt, uri: 'git@github.com:personalrobotics/or_trajopt'}
- git: {local-name: or_urdf, uri: 'git@github.com:personalrobotics/or_urdf'}
- git: {local-name: comps, uri: 'git@github.com:personalrobotics/comps'}
```

## Running HerbPy ##
You use HerbPy in your script by simply calling the ``initialize`` function:

```python
env, robot = herbpy.initialize()
```

By default, this function loads the OpenRAVE plugins necessary to communicate
with HERB's hardware drivers. You can run HerbPy in simulation mode by passing
the option ``sim=True``. In both cases, you can optionally attach a
viewer to the OpenRAVE environment by passing ``attach_viewer=True``.

See [herbpy.herb.initialize]() for the full list of initialization options.

## HerbPy Console ##
HerbPy includes [console.py](), a helper script for launching an interactive
Python environment. Several common [herbpy.herb.initialize]() options are
exposed as command-line arguments:

```bash
rosrun herbpy console.py
rosrun herbpy console.py --sim     # equivalent to sim=True
rosrun herbpy console.py --viewer  # equivalent to attach_viewer=True
```

## Using HERBRobot ##
The robot returned by [herbpy.herb.initialize]() is an OpenRAVE robot of type
[herbpy.herbrobot.HERBRobot](). This object provides access to all of HERB's
hardware-specific functionality:

* ``left_arm``, ``right_arm`` : [prpy.base.wam.WAM]() - Barrett WAM arms
* ``left_hand``, ``right_hand`` : [prpy.base.barretthand.BarrettHand]() - BarrettHand end-effectors
* ``head`` : [herbpy.herbpantilt.HERBPantilt]() - custom pan-tilt head
* ``base`` : [herbpy.herbbase.HerbBase]() - Segway RMP mobile base

### Basic Arm Usage ###

We'll default to the right arm (``robot.left_arm``) in these examples, keep in mind you can do the same things for the right arm too (with ``robot.right_arm``).

The arms can be in two modes, stiff and not stiff. When the arms are not stiff, they are in gravity compensation mode and can be moved around. When the arms are stiff, they cannot be moved, but now planning can be performed.

To make an arm stiff,
```
robot.right_arm.SetStiffness(1)
```

And to put it back into gravity compensation mode
```
robot.right_arm.SetStiffness(0)
```
Make sure the arms are stiff before you begin planning and executing trajectories!

The state of an arm is given by its configuration, or DOFValues (degrees of freedom) values. You can get these by calling

```
robot.right_arm.GetDOFValues()
```

To plan between two configurations of the arm, you can run

```
robot.right_arm.PlanToConfiguration(config)
```

This will compute a plan and then immediately execute it.

There are also named configurations corresponding to common arm configurations. The two most used are 'relaxed_home' and 'home'. You can plan to one of these by running

```
robot.PlanToNamedConfiguration('relaxed_home')
```
or for a single arm
```
robot.right_arm.PlanToNamedConfiguration('relaxed_home')
```

If you do not want to immediately execute a trajectory, you can instead do

```
traj = robot.right_arm.PlanToConfiguration(config, execute=False)
robot.ExecuteTrajectory(traj)
```

You can also add a timeout parameter (in seconds), like so

```
robot.right_arm.PlanToConfiguration(config, timeout=30)
```

If planning or execution fails, the ``PlanToConfiguration`` call will throw an exception

```
try:
    robot.right_arm.PlanToConfiguration(config, timeout=30)
except Exception as e:
    print 'uhoh'
    raise
```

### Planning for the End Effector ###

The most common thing you'll want to do with the arms is move them such that the hand, or end effector, is at a particular position and orientation (a pose) in the world. To plan to a pose, you must first generate a configuration that corresponds to that pose.

```
pose_in_world = ... # must be a 4x4 homogeneous transformation matrix
filter_options = openravepy.IkFilterOptions.CheckEnvCollisions #or 0 for no collision checks
config = robot.right_arm.FindIKSolution(pose_in_world, filter_options) # will return None if no config can be found
robot.right_arm.PlanToConfiguration(config)
```

A shortcut for this is 
```
robot.right_arm.PlanToEndEffectorPose(pose_in_world)
```

You can also move the end effector in an offset from its current position. The following will move .1 meters in the z+ direction (up).
```
distance = .1
direction = [0,0,1]
robot.right_arm.PlanToEndEffectorOffset(direction, distance)
```

Another method moves the end effector in a direction until a force is felt on the arm.
```
min_distance = 0
max_distance = .1
direction = [1,0,0]
robot.right_arm.MoveUntilTouch(direction, min_distance, max_distance)
```

### Hands ###

Each of HERB's hands have 3 fingers. You can change how closed/opened these fingers are, and you can change the spread, which moves the fingers around the axis coming out of the palm of the hand. When the spread is 0, two fingers are across from the other finger. When the spread is 2pi, the fingers are on the same side of the hand next to each other. In the middle, the fingers form a tripod shape.

```
robot.right_arm.hand.MoveHand(spread=0)
```

Then to close the hand, you do
```
robot.right_arm.hand.CloseHand()
```

To open it,
```
robot.right_arm.hand.OpenHand()
```

You can grab an object to mark it as connected to the robot. It will be connected to whatever link of the active part robot was touching the object when the method is called.
```
robot.right_arm.SetActive()
robot.Grab(obj)
```

And to release an object:
```
robot.Release(obj)
```

You can also perform more precise control of the fingers. When a finger is at its 0 position, it is fully opened. At pi, it is fully closed.

For example, to get the hand ready to grasp a cylinder, you could do
```
robot.right_arm.hand.MoveHand(f1=0, f2=0, f3=0, spread=0)
```
Then after you move the hand near the cylinder and you're ready to grab it,
```
robot.right_arm.hand.MoveHand(f1=2, f2=2, f3=2)
```

### Head ###

To move the head so that its facing forwards, do
```
robot.head.MoveTo([0,0])
```
The first entry is rotation about z, the second is rotation about y. For example, to look left and slightly down, 
```
robot.head.MoveTo([math.pi/2, -math.pi/16])
```

### Base ###

To get the position of herb, do
```
robot_pose_in_world = robot.GetTransform()
```

It can be useful to set HERB's position manually, especially in simulation:
```
robot.SetTransform(desired_robot_pose_in_world)
```

To drive HERB forward, do:
```
distanceInMeters = 1
robot.base.Forward(distanceInMeters)
```

To rotate 90 degres,
```
robot.base.rotate(pi/2)
```

You can also plan with the base
```
robot_pose_in_world = ...
robot.base.PlanToBasePose(robot_pose_in_world)
```

### Other Examples ###

[comprehensive documented example of picking up a fuze bottle](examples/graspFuzeBottle.py)
