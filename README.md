HERBPy
======

HerbPy is a Python library for interacting with HERB through OpenRAVE. HERB is
a bimanual mobile manipulator designed and built by the [Personal Robotics Lab]
(https://personalrobotics.ri.cmu.edu) at [Carnegie Mellon University]
(http://www.cmu.edu). HerbPy expands the robot-agnostic helper library [PrPy]
(https://github.com/personalrobotics/prpy) by wrapping HERB-specific functionality.

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

You can browse the API documentation for each of these classes for more detailed
information.

## Examples ##
[Check the lab wiki.](https://wiki.personalrobotics.ri.cmu.edu/lib/exe/fetch.php?media=tutorials:herbpy_tutorial.pdf)

