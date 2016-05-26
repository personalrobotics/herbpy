^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package herbpy
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.0 (2016-05-12)
------------------
* `ros_control` migration. `#63 <https://github.com/personalrobotics/herbpy/issues/63>`_
* Remove environmental variables from `.travis.yml` that are now set on Travis.
* Make test files non-executable to please `nosetest`.
* Move `BarrettHand`, `WAM`, and `WAMRobot` from PrPy to HerbPy.
* Fixed the head and overrode `GetDOFValues` to return the static position.
* Contributors: Clint Liddick, Michael Koval

1.9.0 (2016-05-11)
------------------
* Add named preshapes for the hands. `#81 <https://github.com/personalrobotics/herbpy/issues/81>`_
* Add `yaw_range` parameter to glass grasps. `#80 <https://github.com/personalrobotics/herbpy/issues/80>`_
* Add actions for pitcher pouring YCB task. `#78 <https://github.com/personalrobotics/herbpy/issues/78>`_
* Add actions for cup stacking YCB task. `#77 <https://github.com/personalrobotics/herbpy/issues/77>`_
* Modify RoGuE to use `prpy.util.GetPointFrom`. `#76 <https://github.com/personalrobotics/herbpy/issues/76>`_
* Fix locks in TSR library. `#69 <https://github.com/personalrobotics/herbpy/issues/69>`_
* Refactor lift and pill bottle TSRs. `#68 <https://github.com/personalrobotics/herbpy/issues/68>`_
* Removed or_pushing test and dependency. `#67 <https://github.com/personalrobotics/herbpy/issues/67>`_
* Add `init_node` to enable talker. `#59 <https://github.com/personalrobotics/herbpy/issues/59>`_ `#60 <https://github.com/personalrobotics/herbpy/issues/60>`_
* Add an action for invoking the push planner. `#57 <https://github.com/personalrobotics/herbpy/issues/57>`_
* Removed rosbuild support. `#61 <https://github.com/personalrobotics/herbpy/issues/61>`_
* Added gesture tests. `#58 <https://github.com/personalrobotics/herbpy/issues/58>`_
* Added a gesture interface. `#54 <https://github.com/personalrobotics/herbpy/issues/54>`_
* Switched to blending-only for constrained trajectories. `#43 <https://github.com/personalrobotics/herbpy/issues/43>`_
* Contributors: Clint Liddick, David Butterworth, Jennifer King, Michael Koval, Rachel, Rachel Holladay, Shushman Choudhury, Vinitha Ranganeni

1.8.1 (2015-10-21)
------------------
* Added logic to warn, but not fail, if the perception module cannot be loaded
* Added execute=True flags to actions (`#56 <https://github.com/personalrobotics/herbpy/issues/56>`_)
* Changed pr_ordata to be an optional dependency (`#52 <https://github.com/personalrobotics/herbpy/issues/52>`_)
* Contributors: Jennifer King, Michael Koval, Vinitha Ranganeni

1.8.0 (2015-10-12)
------------------
* Changes for the block sorting demo (`#48 <https://github.com/personalrobotics/herbpy/issues/48>`_)
* Changes to table clearing TSRs
* Added perception pipeline
* Added support of OWD trajectory execution options (`#41 <https://github.com/personalrobotics/herbpy/issues/41>`_)
* Added missing dependencies to package.xml and switched to format 2 (`#40 <https://github.com/personalrobotics/herbpy/issues/40>`_))
* Added lift and place actions (`#37 <https://github.com/personalrobotics/herbpy/issues/37>`_)
* Added robot.Say() function (`#39 <https://github.com/personalrobotics/herbpy/issues/39>`_)
* Fixed README to use pr-rosinstalls
* Fixed broken links in README (`#34 <https://github.com/personalrobotics/herbpy/issues/34>`_)
* Contributors: Aaron Johnson, Clint Liddick, Jennifer King, Michael Koval, Pras Velagapudi, Rachel Holladay, Shushman, Vinitha Ranganeni

1.7.1 (2015-06-04)
------------------
* Updating bowl tsr. Updating default detection frame to match latest perception changes.
* Correction to tsr values for pointing tsr
* Contributors: Jennifer King, Michael Koval, Rachel Holladay

1.7.0 (2015-06-01)
------------------
* Adding ability to add extra padding around handles picking a point on the tray
* Updating tray TSR to correctly handle negative distance when lifting tray
* Adding a function to detect objects to herbpy
* Contributors: Jennifer King

1.6.1 (2015-05-01)
------------------
* Updating tray TSR
* Contributors: Jennifer King

1.6.0 (2015-05-01)
------------------
* Removing reference to BiRRT planner
* Lots of cleanup. Removing dead code. Removing support for fuerte.
* Remove attempt at loading TSRs from yaml
* Changed the planning pipeline, retimer, and smoother.
* Adding pull TSR to tray
* Changing planning pipeline. Setting smoother and retimer to HauserParabolicSmoother.
* Removed vision_sim and MarkerSensorSystem.
* Explicitly pass iktype as a named parameter.
* Contributors: Jennifer King, Michael Koval

1.5.1 (2015-04-07)
------------------
* Small modifications to tray TSR as a result of robot testing
* Contributors: Jennifer King

1.5.0 (2015-04-07)
------------------
* Adding tray to default list of TSRs that get loaded
* Merge pull request `#19 <https://github.com/personalrobotics/herbpy/issues/19>`_ from personalrobotics/feature/issue18
  Enable PlanToEndEffectorPose and PlanToEndEffectorOffset on TSR planners.
* Adding PlanToEndEffectorPose and PlanToEndEffectorOffset to methods in MethodMask
* Adding some lateral tolernace to tray point on tsr
* Merge branch 'feature/action_library'
  Conflicts:
  src/herbpy/tsr/fuze.py
* Changed active manipulator settig to be done within RobotStateSaver. Modified use of render flat to just pass it along to render functions. Fixed some env locking problems in HerbGrasp
* Moving tray tsr definition into herbpy
* Updating glass TSRs to have push_grasp and grasp TSR to have same fundamental code base
* Adding push_grasp action to fuze. Fixing grasping to allow tsrlist and render as options to Grasp and PushGrasp actions
* Merge pull request `#17 <https://github.com/personalrobotics/herbpy/issues/17>`_ from personalrobotics/feature/PlannerRefactor
  Updated the default planning pipeline
* Renamed planner names to match PrPy.
* Disable TrajOpt logging.
* Updated planner order.
* Explicitly pass delegate planners.
* Changed HERB's default list of planners.
* Merge pull request `#13 <https://github.com/personalrobotics/herbpy/issues/13>`_ from personalrobotics/feature/pointing_tsr
  Added a TSR for pointing at Fuze bottles.
* robot wrapper
* Cleaning up call to MoveHand in grasping action
* Renaming actionlibrary to actions
* Updating push grasp code. Removing shortcuts accidently committed to example code.
* Small bugfixes to Grasp action. Adding default loading of herb tsrs and actions. Adding fuze grasping tsr. Updating example to use the grasp action.
* Adding action library to herb
* Implementing grasping actions
* first draft of pointing tsr
* Contributors: Jennifer King, Michael Koval, Rachel Holladay

1.4.0 (2015-03-30)
------------------
* Calling ExecutePath instead of ExecuteTrajectory in MoveTo
* Transmission ratio calibration script
* Adding CHOMP back to planner list to allow access to OptimizeTrajectory function. Slight modification to placement TSRs for bowl and glass - allows for stacking objects.
* Set HERB's acceleration limits (not set by URDF).
* Removed CHOMP as a default planner.
* Updating ordering of planners. Adding workspace planner
* added more documentation and an example
* New transmission ratios.
* Implemented set_one_direction.
* Implemented set_one_angle_offset.
* Added a skeleton script to calibrate transmission ratios.
* Contributors: Evan, Evan Shapiro, Jennifer King, Michael Koval

1.3.0 (2015-02-10)
------------------
* Updating plate, bowl and glass tsrs
* Adding back in chomp planner
* Adding head tf publisher
* Added RaveInitialize to fix --debug flag.
* Switched to "rviz" as the default viewer.
* Adding protection against an empty or missing tsrs.yaml. This is not required.
* Now selectively loads planners and does not fail when it can't find one.
* Explicitly import sub-modules from prpy.base.
* Contributors: Aaron Walsman, Jennifer King, Michael Koval, Pras Velagapudi

1.2.0 (2014-12-12)
------------------
* Deleting transient save file.
* Contributors: Pras Velagapudi

1.1.0 (2014-12-11)
------------------
* Added TSRLibrary to HERB.
* Adding TSRs for the pitcker, the block, and an "upright" constraint.
* Enabled CHOMP and OMPL.
* Contributors: Aaron Walsman, Jennifer King, Michael Koval, Pras Velagapudi

1.0.1 (2014-10-10 14:49)
------------------------
* Correctly load YAML files from Catkin install spaces.
* Moved SBPL primitive scripts into the scripts dir.
* Contributors: Mike Koval

1.0.0 (2014-10-10 11:36)
------------------------
* Initial release.
