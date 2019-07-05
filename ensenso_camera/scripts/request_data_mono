#!/usr/bin/env python
import rospy
import sys

import actionlib
from actionlib_msgs.msg import GoalStatus
from ensenso_camera_msgs.msg import RequestDataMonoAction, RequestDataMonoGoal


def main():
    loop_rate = rospy.get_param("~rate", 10)
    timeout = rospy.get_param("~timeout", 60)

    goal = RequestDataMonoGoal()
    goal.parameter_set = rospy.get_param("~parameter_set", "")

    goal.request_raw_images = rospy.get_param("~raw_images", True)
    goal.request_rectified_images = rospy.get_param("~rectified_images", True)

    goal.publish_results = True

    request_data_client = actionlib.SimpleActionClient("request_data", RequestDataMonoAction)
    if not request_data_client.wait_for_server(rospy.Duration(timeout)):
        rospy.logerr("The camera node is not running!")
        sys.exit()

    rate = rospy.Rate(loop_rate)
    while not rospy.is_shutdown():
        request_data_client.send_goal(goal)
        request_data_client.wait_for_result()

        if request_data_client.get_state() != GoalStatus.SUCCEEDED:
            rospy.logwarn("Action was not successful.")
        else:
            result = request_data_client.get_result()
            if result.error.code != 0:
                rospy.logerr("Error {}: {}".format(result.error.code, result.error.message))

        rate.sleep()


if __name__ == "__main__":
    try:
        rospy.init_node("ensenso_camera_request_data")
        main()
    except rospy.ROSInterruptException:
        pass
