Due to in bug in ROS2 foxy that fails the launch system to start if an empty string is provided as default argument for
a launch argument, the launch files currently contain a workaround that allows providing empty strings as default
argument.

The workaround consits of the following:
- set empty default values to ros2_launch.EMPTY_STRING alias enenseo.EMPTY_STRING
- define the launch arguments in `generate_launch_description`
- use `launch_setup` as OpaqueFunction for the rest of the code, because this function can access the context object and
  thus access the values of the launch arguments
- within this function we can now check if the arguments are ensenso.EMPTY_STING and if so return a true empty string by
  performing the substition with `.perform(context)`

Bug description:
https://github.com/introlab/rtabmap_ros/issues/725

Proposed solution with `launch_setup` and `.perform(context)`:
https://answers.ros.org/question/340705/access-launch-argument-in-launchfile-ros2/
https://github.com/jrgnicho/collaborative-robotic-sanding/blob/3902e4f0e76bde226b18a997fd60fc30e1961212/crs_application/launch/perception.launch.py#L21

Future:
If the bug is fixed, we can go back to the simple launch file and replace `ensenso.EMPTY_STRING` with an true empty
python string.
