#pragma once

#include "ensenso_camera/ros2/logging.h"
#include "ensenso_camera/ros2/node_handle.h"
#include "ensenso_camera/ros2/core.h"

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
 * An action server that remembers a queue of goals that still have to be processed. This server will never reject or
 * cancel any requests on its own.
 *
 * The API is the same as for the SimpleActionServer.
 */
template <class ActionSpec>
class QueuedActionServer
{
public:
  ACTION_DEFINITION(ActionSpec)
  using GoalHandle = typename actionlib::ActionServer<ActionSpec>::GoalHandle;

  using ExecuteCallback = ensenso::std::function<void(GoalConstPtr const&)>;

public:
  QueuedActionServer(ensenso::ros::NodeHandle& nh, std::string const& name, ExecuteCallback callback,
                     bool autoStart = false)
    : nh(nh), name(name), callback(callback)
  {
    actionServer = ensenso::std::make_unique<actionlib::ActionServer<ActionSpec>>(nh, name, false);
    actionServer->registerGoalCallback(ensenso::std::bind(&QueuedActionServer::onGoalReceived, this, _1));
    actionServer->registerCancelCallback(ensenso::std::bind(&QueuedActionServer::onCancelReceived, this, _1));

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

  bool isPreemptRequested()
  {
    return preemptRequested;
  }

  void setSucceeded(Result result = Result())
  {
    std::lock_guard<std::mutex> lock(mutex);

    ENSENSO_DEBUG_NAMED(getLoggerName(), "Setting the current goal as succeeded.");
    currentGoal.setSucceeded(result);
  }

  void setAborted(Result result = Result())
  {
    std::lock_guard<std::mutex> lock(mutex);

    ENSENSO_DEBUG_NAMED(getLoggerName(), "Setting the current goal as aborted.");
    currentGoal.setAborted(result);
  }

  void setPreempted(Result result = Result())
  {
    std::lock_guard<std::mutex> lock(mutex);

    ENSENSO_DEBUG_NAMED(getLoggerName(), "Setting the current goal as canceled.");
    currentGoal.setCanceled(result);
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
    return ros::this_node::getName() + "/QueuedActionServer/" + name;
  }

  void onGoalReceived(GoalHandle goal)
  {
    ENSENSO_DEBUG_NAMED(getLoggerName(), "Received a new goal.");
    std::lock_guard<std::mutex> lock(mutex);

    goalQueue.push(goal);
    loopCondition.notify_one();
  }

  void onCancelReceived(GoalHandle goal)
  {
    ENSENSO_DEBUG_NAMED(getLoggerName(), "Received a cancel request.");
    std::lock_guard<std::mutex> lock(mutex);

    if (goal == currentGoal)
    {
      // The goal is already being executed. We set the preemption flag and the user is responsible for polling it with
      // isPreemptRequested() and canceling his action handler with setPreempted().
      preemptRequested = true;
    }
    else
    {
      // The goal is in the queue. We cancel it and ignore it later, when we extract new goals from the queue.
      goal.setCanceled(Result(), "Goal was canceled by the user.");
    }
  }

  void loop()
  {
    std::unique_lock<std::mutex> lock(mutex);

    while (nh.ok() && !shutdownRequested)
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
          // The goal might have already been canceled by the cancel callback above.
          continue;
        }
        else
        {
          ENSENSO_DEBUG_NAMED(getLoggerName(), "Accepting a new goal.");
          currentGoal.setAccepted();

          preemptRequested = false;

          lock.unlock();
          callback(currentGoal.getGoal());
          lock.lock();

          if (currentGoal.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE ||
              currentGoal.getGoalStatus().status == actionlib_msgs::GoalStatus::PREEMPTING)
          {
            ENSENSO_WARN_NAMED(getLoggerName(),
                               "Your action handler did not set the goal to a terminal state. Aborting it for now.");
            setAborted(Result());
          }
        }
      }

      loopCondition.wait_for(lock, std::chrono::milliseconds(100));
    }
  }

private:
  ensenso::ros::NodeHandle nh;
  std::string name;
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
