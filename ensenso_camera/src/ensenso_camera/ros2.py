import os
import sys

# ----------------------------------------------------------------------------------------------------------------------
# Common ROS1 and ROS2 constants and methods
# ----------------------------------------------------------------------------------------------------------------------

SERVER_TIMEOUT_ERROR_MESSAGE = "The camera node is not running!"


def import_from_module(module_name, attribute_name):
    try:
        return getattr(__import__(module_name, globals(), locals(), [attribute_name], 0), attribute_name)
    except AttributeError:
        print("Error while importing class %s from module %s" % (attribute_name, module_name))
        return None
    except ImportError as e:
        print("Error while importing class %s: %s" % (attribute_name, e))
        return None


def is_ros2():
    return os.environ["ROS_VERSION"] == "2"


def format_error(error, note=None):
    msg = "Error {}: {}".format(error.code, error.message)
    if note:
        msg = "{} ({})".format(note.strip(), msg)
    return msg


# ----------------------------------------------------------------------------------------------------------------------
# ROS2
# ----------------------------------------------------------------------------------------------------------------------
if is_ros2():
    import threading
    import time

    import rclpy

    from action_msgs.msg import GoalStatus
    from rclpy.action import ActionClient
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor

    def import_action(package_name, action_name):
        return import_from_module(package_name + ".action", action_name)

    class ActionResponse:
        def __init__(self, status, result=None, timeout=False):
            self._result = result
            self._status = status
            self._timeout = timeout

        def get_result(self):
            return self._result

        def successful(self):
            return self._status == GoalStatus.STATUS_SUCCEEDED

        def timeout(self):
            return self._timeout

    class ActionHandler:
        def __init__(self, node, client, goal, feedback_callback=None):
            self._node = node
            self._client = client
            self._event = threading.Event()
            self._goal = goal
            self._feedback_callback = feedback_callback
            self._response = None
            self._result = None

        def send_goal(self):
            self._event.clear()
            send_goal_future = self._client.send_goal_async(self._goal, feedback_callback=self._feedback_callback)
            send_goal_future.add_done_callback(self.response_callback)

        def wait_for_response(self, timeout_sec=None):
            # Separate sending goal and waiting for response so that we can handle several clients at once by sending
            # all the goals first and then waiting until all clients have received the response.
            self._event.wait(timeout_sec)
            self._response = ActionResponse(self._result.status, self._result.result)

        def response_callback(self, future):
            goal_handle = future.result()
            # ROS2 does not provide means to check whether the retrieval of an action result timed out, but it provides
            # an `accepted()` method.
            if not goal_handle.accepted:
                self._response = ActionResponse(GoalStatus.STATUS_ABORTED)
                return
            get_result_future = goal_handle.get_result_async()
            get_result_future.add_done_callback(self.result_callback)

        def result_callback(self, future):
            self._result = future.result()
            self._event.set()

        def get_response(self):
            return self._response

    def Duration(sec):
        return sec

    def sleep(node, secs):
        frequency = 1.0 / secs
        rate = node.create_rate(frequency)
        rate.sleep()

    def wrap_main_function(main, node_name):
        try:
            main(node_name)
        except Exception as e:
            raise e

    def create_node(name, args=sys.argv):
        rclpy.init(args=args)
        node = rclpy.create_node(name)
        executor = MultiThreadedExecutor()
        node.thread = threading.Thread(target=rclpy.spin, args=(node, executor), daemon=True)
        node.callback_group = ReentrantCallbackGroup()
        node.spinning = True
        node.thread.start()
        return node

    def ok():
        return rclpy.ok()

    def shutdown(node):
        node.destroy_node()
        rclpy.shutdown()
        if node.spinning:
            node.thread.join()

    def get_param(node, name, value=None):
        return node.declare_parameter(name, value).value

    def create_publisher(node, msg_type, topic, queue_size=1):
        return node.create_publisher(msg_type, topic, queue_size)

    def create_subscription(node, msg_type, topic, callback, qos_profile=10):
        return node.create_subscription(msg_type, topic, callback, qos_profile)

    def wait_for_server(node, client, timeout_sec=None, exit=False):
        """
        Returns True if the client established a server connection in the given amount of time, False otherwise.
        """
        node.get_logger().info("Connecting to action server {} ...".format(client._action_name))
        if not client.wait_for_server(timeout_sec):
            node.get_logger().error(SERVER_TIMEOUT_ERROR_MESSAGE)
            if exit:
                sys.exit()
            return False
        node.get_logger().info("Connected!")
        return True

    def wait_for_servers(node, clients, timeout_sec=None, exit=False):
        """
        Returns True if all clients established a server connection in the given amount of time, False otherwise.
        """
        for client in clients:
            if not wait_for_server(node, client, timeout_sec, exit):
                return False
        return True

    # ROS2 provides two methods to send a goal:
    # (1) 'send_goal_async`: This method sends the goal and registers three callbacks, one for when the feedback is
    #     ready, one for when the response (as a goal handle, which allows to check the GoalStatus of the action) is
    #     ready and one for when the result is ready
    # (1) `send_goal`: This method waits for the result to be ready and then returns the final result (but does not
    #     provide a way to check the GoalStatus, since that can only be done with a goal handle)
    # Since we need the goal status, the asynchronous variant has to be used!
    #
    # Sources:
    # https://docs.ros2.org/foxy/api/rclpy/api/actions.html
    # https://docs.ros.org/en/foxy/Tutorials/Actions/Writing-a-Py-Action-Server-Client.html

    def send_action_goal(node, client, goal, feedback_callback=None, timeout_sec=None):
        """
        Sends a client goal, waits for the response and returns it.
        """
        action_handler = ActionHandler(node, client, goal, feedback_callback)
        action_handler.send_goal()
        action_handler.wait_for_response(timeout_sec)
        return action_handler.get_response()

    def send_action_goals(node, clients, goals, feedback_callback=None, timeout_sec=None):
        """
        Sends multiple client goals, waits for the responses and returns them. Since all goals are sent first before
        waiting, this approach is faster than repeated sending-waiting.
        """
        action_handlers = []
        for client, goal in zip(clients, goals):
            action_handlers.append(ActionHandler(node, client, goal, feedback_callback))
            action_handlers[-1].send_goal()

        responses = []
        for action_handler in action_handlers:
            action_handler.wait_for_response(timeout_sec)
            responses.append(action_handler.get_response())

        return responses

    def create_action_client(node, action_name, action):
        return ActionClient(node, action, action_name, callback_group=node.callback_group)

    def execute_at_rate(node, func, rate_in_hz):
        # As explained in https://answers.ros.org/question/358343/, we have to spin because Rate in ROS2 registers a
        # callback with a ROS timer and without spinning the callback is never executed and thus the timer will never
        # wake up and trigger.
        try:
            rate = node.create_rate(rate_in_hz)
            while rclpy.ok():
                func()
                rate.sleep()
        except KeyboardInterrupt:
            pass
        except Exception as e:
            raise e
        finally:
            shutdown(node)


# ----------------------------------------------------------------------------------------------------------------------
# ROS1
# ----------------------------------------------------------------------------------------------------------------------
else:
    import rospy

    import actionlib

    from actionlib_msgs.msg import GoalStatus

    class ActionWrapper:
        """Immitate ROS2 action object style."""

        def __init__(self, action_class, goal_class):
            self._action_class = action_class
            self._goal_class = goal_class

        @property
        def Action(self):
            """Return action class."""
            return self._action_class

        @property
        def Goal(self):
            """Return goal class (in order to be able to access the constants)."""
            return self._goal_class

    def import_action(package_name, action_name):
        _package_name = package_name + ".msg"
        _action_name = action_name + "Action"
        _action_goal_name = action_name + "Goal"
        return ActionWrapper(
            import_from_module(_package_name, _action_name), import_from_module(_package_name, _action_goal_name)
        )

    class ActionResponse:
        def __init__(self, status, result=None, timeout=False):
            self._result = result
            self._status = status
            self._timeout = timeout

        def get_result(self):
            return self._result

        def successful(self):
            return self._status == GoalStatus.SUCCEEDED

        def timeout(self):
            return self._timeout

    class Clock:
        def now(self):
            return rospy.Time.now()

    def Duration(sec):
        return rospy.Duration(sec)

    def sleep(node, secs):
        rospy.sleep(secs)

    class Logger:
        def debug(self, msg):
            rospy.logdebug(msg)

        def warn(self, msg):
            rospy.logwarn(msg)

        def info(self, msg):
            rospy.loginfo(msg)

        def error(self, msg):
            rospy.logerr(msg)

    class Node:
        def __init__(self):
            self._logger = Logger()
            self._clock = Clock()

        def get_logger(self):
            return self._logger

        def get_clock(self):
            return self._clock

    def wrap_main_function(main, node_name):
        try:
            main(node_name)
        except rospy.ROSInterruptException:
            pass

    def create_node(name, args=None):
        rospy.init_node(name)
        node = Node()
        return node

    def spin(node):
        pass

    def shutdown(node):
        pass

    def get_param(_, name, default=rospy.client._unspecified):
        # Use the tilde as prefix in order to make the parameter private to the node.
        return rospy.get_param("~" + name, default)

    def create_publisher(_, msg_type, topic, queue_size=1):
        return rospy.Publisher(topic, msg_type, queue_size=queue_size)

    def create_subscription(_, msg_type, topic, callback):
        return rospy.Subscriber(topic, msg_type, callback)

    def wait_for_server(node, client, timeout_sec=None, exit=True):
        """
        Returns True if the client established a server connection in the given amount of time, False otherwise.
        """
        timeout = rospy.Duration() if timeout_sec is None else rospy.Duration(timeout_sec)
        node.get_logger().info("Connecting to action server {} ...".format(client.action_client.ns))
        if not client.wait_for_server(timeout):
            node.get_logger().error(SERVER_TIMEOUT_ERROR_MESSAGE)
            if exit:
                sys.exit()
            return False
        node.get_logger().info("Connected!")
        return True

    def wait_for_servers(node, clients, timeout_sec=None, exit=True):
        """
        Returns True if all clients established a server connection in the given amount of time, False otherwise.
        """
        for client in clients:
            if not wait_for_server(node, client, timeout_sec, exit):
                return False
        return True

    def get_timeout_settings_for_send_action_goal(timeout_sec):
        # For the unittests the knowledge about the result timeout must be integrated into the responses.
        check_timeout = True if timeout_sec is not None else False
        # Use default zero timeout if no timeout is given, else use the given timeout.
        timeout_duration = rospy.Duration() if timeout_sec is None else timeout_sec
        return check_timeout, timeout_duration

    def send_action_goal(_, client, goal, feedback_callback=None, timeout_sec=None):
        """
        Sends a client goal, waits for the response and returns it.
        see: http://docs.ros.org/en/diamondback/api/actionlib/html/simple__action__client_8py_source.html
        """
        check_timeout, timeout_duration = get_timeout_settings_for_send_action_goal(timeout_sec)

        client.send_goal(goal, feedback_cb=feedback_callback)

        # wait_for_result returns True if the goal finished within the allocated time.
        timeout = not client.wait_for_result(timeout_duration)

        if check_timeout and timeout:
            return ActionResponse(client.get_state(), timeout=True)
        return ActionResponse(client.get_state(), client.get_result())

    def send_action_goals(_, clients, goals, feedback_callback=None, timeout_sec=None):
        """
        Sends multiple client goals, waits for the responses and returns them. Since all goals are sent first before
        waiting, this approach is faster than repeated sending-waiting.
        """
        check_timeout, timeout_duration = get_timeout_settings_for_send_action_goal(timeout_sec)

        responses = []

        for client, goal in zip(clients, goals):
            client.send_goal(goal, feedback_cb=feedback_callback)

        for client in clients:
            # wait_for_result returns True if the goal finished within the allocated time.
            timeout = not client.wait_for_result(timeout_duration)
            if check_timeout and timeout:
                responses.append(ActionResponse(client.get_state(), timeout=True))
            else:
                responses.append(ActionResponse(client.get_state(), client.get_result()))

        return responses

    def send_goal(client, feedback_callback=None, timeout=None):
        pass

    def send_multiple_goals(clients, feedback_callback=None, timeout=None):
        pass

    def execute_at_rate(_, func, rate_in_hz):
        rate = rospy.Rate(rate_in_hz)
        while not rospy.is_shutdown():
            func()
            rate.sleep()

    def create_action_client(_, name, action_wrapper):
        return actionlib.SimpleActionClient(name, action_wrapper.Action)
