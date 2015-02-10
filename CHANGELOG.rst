^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package herbpy
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
