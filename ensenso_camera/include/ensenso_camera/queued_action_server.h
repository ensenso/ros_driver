#pragma once

#include "ensenso_camera/helper.h"

#include <actionlib/action_definition.h>
#include <actionlib/server/action_server.h>
#include <ros/ros.h>

#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>

/**
 * An action server that remembers a queue of goals that still have to be
 * processed. This server will never reject or cancel any requests on its own.
 *
 * The API is the same as for the SimpleActionServer.
 */
template <class ActionSpec>
class QueuedActionServer
{
public:
  ACTION_DEFINITION(ActionSpec)
  using GoalHandle = typename actionlib::ActionServer<ActionSpec>::GoalHandle;

  using ExecuteCallback = boost::function<void(GoalConstPtr const&)>;

public:
  QueuedActionServer(ros::NodeHandle nodeHandle, std::string const& name, ExecuteCallback callback,
                     bool autoStart = false)
    : nodeHandle(nodeHandle), callback(callback)
  {
    actionServer = ::make_unique<actionlib::ActionServer<ActionSpec>>(nodeHandle, name, false);
    actionServer->registerGoalCallback(boost::bind(&QueuedActionServer::onGoalReceived, this, _1));
    actionServer->registerCancelCallback(boost::bind(&QueuedActionServer::onCancelReceived, this, _1));

    if (autoStart)
    {
      start();
    }
  }

  ~QueuedActionServer()
  {
    shutdown();
  }

  void start()
  {
    actionServer->start();
    loopThread = std::thread([this] { loop(); });  // NOLINT
  }

  void shutdown()
  {
    if (loopThread.joinable())
    {
      shutdownRequested = true;
      loopThread.join();
    }
  }

  bool isPreemptRequested()
  {
    return preemptRequested;
  }

  void setSucceeded(Result const& result = Result(), std::string const& text = "")
  {
    std::lock_guard<std::mutex> lock(mutex);

    ROS_DEBUG_NAMED("actionlib", "Setting the current goal as succeeded.");
    currentGoal.setSucceeded(result, text);
  }

  void setAborted(Result const& result = Result(), std::string const& text = "")
  {
    std::lock_guard<std::mutex> lock(mutex);

    ROS_DEBUG_NAMED("actionlib", "Setting the current goal as aborted.");
    currentGoal.setAborted(result, text);
  }

  void setPreempted(Result const& result = Result(), std::string const& text = "")
  {
    std::lock_guard<std::mutex> lock(mutex);

    ROS_DEBUG_NAMED("actionlib", "Setting the current goal as canceled.");
    currentGoal.setCanceled(result, text);
  }

  void publishFeedback(FeedbackConstPtr const& feedback)
  {
    currentGoal.publishFeedback(*feedback);
  }

  void publishFeedback(Feedback const& feedback)
  {
    currentGoal.publishFeedback(feedback);
  }

private:
  void onGoalReceived(GoalHandle goal)
  {
    ROS_DEBUG_NAMED("actionlib", "Received a new goal.");
    std::lock_guard<std::mutex> lock(mutex);

    goalQueue.push(goal);
    loopCondition.notify_one();
  }

  void onCancelReceived(GoalHandle goal)
  {
    ROS_DEBUG_NAMED("actionlib", "Received a cancel request.");
    std::lock_guard<std::mutex> lock(mutex);

    if (goal == currentGoal)
    {
      // The goal is already being executed. We set the preemption flag
      // and the user is responsible for polling it and canceling his
      // action handler.
      preemptRequested = true;
    }
    else
    {
      // The goal is in the queue. We cancel it and ignore it later, when
      // we extract new goals from the queue.
      goal.setCanceled(Result(), "Goal was canceled by the user.");
    }
  }

  void loop()
  {
    std::unique_lock<std::mutex> lock(mutex);
    while (nodeHandle.ok() && !shutdownRequested)
    {
      if (!goalQueue.empty())
      {
        currentGoal = goalQueue.front();
        goalQueue.pop();

        if (!currentGoal.getGoal())
        {
          continue;
        }
        else if (currentGoal.getGoalStatus().status == actionlib_msgs::GoalStatus::RECALLING)
        {
          currentGoal.setCanceled(Result(), "Goal was canceled by the user.");
        }
        else if (currentGoal.getGoalStatus().status != actionlib_msgs::GoalStatus::PENDING)
        {
          // The goal might have already been canceled by the cancel
          // callback above.
          continue;
        }
        else
        {
          ROS_DEBUG_NAMED("actionlib", "Accepting a new goal.");
          currentGoal.setAccepted();

          preemptRequested = false;

          lock.unlock();
          callback(currentGoal.getGoal());
          lock.lock();

          if (currentGoal.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE ||
              currentGoal.getGoalStatus().status == actionlib_msgs::GoalStatus::PREEMPTING)
          {
            ROS_WARN_NAMED("actionlib", "Your action handler did not set the "
                                        "goal to a terminal state. Aborting it "
                                        "for now.");
            setAborted(Result(), "Aborted, because the user did not set the "
                                 "goal to a terminal state.");
          }
        }
      }

      loopCondition.wait_for(lock, std::chrono::milliseconds(100));
    }
  }

private:
  ros::NodeHandle nodeHandle;
  std::unique_ptr<actionlib::ActionServer<ActionSpec>> actionServer;

  std::thread loopThread;

  std::mutex mutex;
  std::condition_variable loopCondition;

  ExecuteCallback callback;

  GoalHandle currentGoal;
  std::queue<GoalHandle> goalQueue;

  // Whether the user canceled the goal that is currently being executed.
  bool preemptRequested = false;

  volatile bool shutdownRequested = false;
};
