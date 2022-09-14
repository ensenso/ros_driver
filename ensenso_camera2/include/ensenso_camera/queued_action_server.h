#pragma once

#include "ensenso_camera/ros2/logging.h"
#include "ensenso_camera/ros2/node_handle.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include <chrono>
#include <condition_variable>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>

/**
 * An action server that remembers a queue of goals that still have to be processed. This server will never reject or
 * cancel any requests on its own.
 *
 * The API is the same as for the queued_action_server for ROS1.
 */
template <class ActionT>
class QueuedActionServer
{
public:
  using ActionServerSharedPtr = typename rclcpp_action::Server<ActionT>::SharedPtr;

  using Goal = typename ActionT::Goal;
  using GoalConstSharedPtr = std::shared_ptr<const Goal>;

  using GoalHandle = typename rclcpp_action::ServerGoalHandle<ActionT>;
  using GoalHandleSharedPtr = std::shared_ptr<GoalHandle>;

  using GoalUUID = rclcpp_action::GoalUUID;

  using Result = typename ActionT::Result;

  using Feedback = typename ActionT::Feedback;
  using FeedbackConstSharedPtr = std::shared_ptr<const Feedback>;

  // using ExecuteCallback = ensenso::std::function<void(GoalConstSharedPtr)>;
  using ExecuteCallback = std::function<void(GoalConstSharedPtr)>;

public:
  QueuedActionServer(ensenso::ros::NodeHandle nh, std::string const& name, ExecuteCallback callback,
                     bool autoStart = false)
    : nh(nh), name(name), callback(callback)
  {
    this->actionServer = rclcpp_action::create_server<ActionT>(
        nh->get_base_interface(), nh->get_clock_interface(), nh->get_logging_interface(), nh->get_waitables_interface(),
        name, std::bind(&QueuedActionServer::onGoalReceived, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&QueuedActionServer::onCancelReceived, this, std::placeholders::_1),
        std::bind(&QueuedActionServer::onGoalAccepted, this, std::placeholders::_1));

    if (autoStart)
    {
      start();
    }
  }

public:
  ~QueuedActionServer()
  {
    shutdown();
  }

  void start()
  {
    loopThread = std::thread([this] { loop(); });  // NOLINT
  }

  bool isPreemptRequested()
  {
    return preemptRequested;
  }

  void setSucceeded(Result&& result = Result())
  {
    std::lock_guard<std::mutex> lock(mutex);

    ENSENSO_DEBUG_NAMED(getLoggerName(), "Setting the current goal as succeeded.");
    currentGoalHandle->succeed(std::make_shared<Result>(std::move(result)));
  }

  void setAborted(Result&& result = Result())
  {
    std::lock_guard<std::mutex> lock(mutex);

    ENSENSO_DEBUG_NAMED(getLoggerName(), "Setting the current goal as aborted.");
    currentGoalHandle->abort(std::make_shared<Result>(std::move(result)));
  }

  void setPreempted(Result&& result = Result())
  {
    std::lock_guard<std::mutex> lock(mutex);

    ENSENSO_DEBUG_NAMED(getLoggerName(), "Setting the current goal as canceled.");
    currentGoalHandle->canceled(std::make_shared<Result>(std::move(result)));
  }

  void publishFeedback(FeedbackConstSharedPtr const& feedback)
  {
    currentGoalHandle->publish_feedback(feedback);
  }

  void publishFeedback(Feedback const& feedback)
  {
    currentGoalHandle->publish_feedback(std::make_shared<Feedback>(feedback));
  }

private:
  void shutdown()
  {
    if (loopThread.joinable())
    {
      shutdownRequested = true;
      loopThread.join();
    }
  }

  std::string getLoggerName()
  {
    return ensenso::ros::get_node_name(nh) + "/QueuedActionServer/" + name;
  }

private:
  rclcpp_action::GoalResponse onGoalReceived(GoalUUID const& uuid, GoalConstSharedPtr goal)
  {
    ENSENSO_DEBUG_NAMED(getLoggerName(), "Received a new goal.");
    std::lock_guard<std::mutex> lock(mutex);

    (void)uuid;
    (void)goal;

    // We accept the goal but defer the execution until the goal is next in the queue.
    return rclcpp_action::GoalResponse::ACCEPT_AND_DEFER;
  }

  rclcpp_action::CancelResponse onCancelReceived(GoalHandleSharedPtr const goalHandle)
  {
    ENSENSO_DEBUG_NAMED(getLoggerName(), "Received a cancel request.");
    std::lock_guard<std::mutex> lock(mutex);

    if (goalHandle == currentGoalHandle)
    {
      // The goal is already being executed. We set the preemption flag and the user is responsible for polling it with
      // isPreemptRequested() and canceling his action handler with setPreempted().
      preemptRequested = true;
    }
    else
    {
      // The goal is in the queue. We cancel it and ignore it later, when we extract new goals from the queue.
      ENSENSO_DEBUG_NAMED(getLoggerName(), "Goal was canceled by the user.");
      goalHandle->canceled(std::make_shared<Result>());
    }

    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void onGoalAccepted(GoalHandleSharedPtr const goalHandle)
  {
    ENSENSO_DEBUG_NAMED(getLoggerName(), "Accepted the new goal.");
    std::lock_guard<std::mutex> lock(mutex);

    goalHandleQueue.push(goalHandle);
    loopCondition.notify_one();
  }

  void loop()
  {
    std::unique_lock<std::mutex> lock(mutex);

    while (rclcpp::ok() && !shutdownRequested)
    {
      if (!goalHandleQueue.empty())
      {
        currentGoalHandle = goalHandleQueue.front();
        goalHandleQueue.pop();

        if (!currentGoalHandle->get_goal())
        {
          continue;
        }
        else if (currentGoalHandle->is_canceling())
        {
          // The goal is currently being canceled.
          continue;
        }
        else
        {
          ENSENSO_DEBUG_NAMED(getLoggerName(), "Executing a new goal.");
          currentGoalHandle->execute();

          preemptRequested = false;

          lock.unlock();
          callback(currentGoalHandle->get_goal());
          lock.lock();

          if (currentGoalHandle->is_active())
          {
            ENSENSO_WARN_NAMED(getLoggerName(),
                               "Your action handler did not set the goal to a terminal state. Aborting it for now.");
            setAborted();
          }
        }
      }

      loopCondition.wait_for(lock, std::chrono::milliseconds(100));
    }
  }

private:
  ensenso::ros::NodeHandle nh;
  std::string name;
  ExecuteCallback callback;

  ActionServerSharedPtr actionServer;
  std::queue<GoalHandleSharedPtr> goalHandleQueue;
  GoalHandleSharedPtr currentGoalHandle;

  std::thread loopThread;
  std::mutex mutex;
  std::condition_variable loopCondition;

  // Whether the user canceled the goal that is currently being executed.
  bool preemptRequested = false;

  volatile bool shutdownRequested = false;
};
