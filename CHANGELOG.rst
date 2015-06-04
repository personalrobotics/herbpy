^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package herbpy
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
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
