^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package segbot_gazebo_plugins
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.2 (2013-07-16)
------------------
* cleaned up and alphabetized cmake infrastructure and package description
* added missing gencpp dependency, ensuring build reliability
* code cleanup + removal of redundant diff drive plugin (migrated to gazebo_plugins)
* now works with gazebo prerelease 1.9.4 and SDF 1.4. This should hopefully allow a successful build on the buildfarm

0.1.1 (2013-07-15)
------------------
* closes `#4 <https://github.com/utexas-bwi/segbot_simulator/issues/4>`_. This was also causing a buildfarm failure

0.1.0 (2013-07-10)
------------------
* upgraded test world to SDF 1.4
* catkinization complete
* fixed cmake and launch errors for gazebo_ros_pkgs migration
* migrating to gazebo_ros_pkgs
* removed redundant function declaration and usage
* improved cmake config
* moved ObjectControllerPlugin to segbot_gazebo_plugins from bwi_gazebo_entities
* moved gazebo ros video plugin to segbot_gazebo_plugins
* cleaned up the full differential drive plugin
* getting plugin to compile with gazebo 1.5
* fixing API breakages
* the simple plugin now looks for the map to avoid collisions
* merged the 2 segbot plugins to remove code redundancy, added setting the updateRate, a number of bug fixes to the plugin
* a working test gazebo environment
* removed old compile rules for BWI controllers that should not be in the segbot package, and fix the plugins dependency
* importing relecant files from svn repository
