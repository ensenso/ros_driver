#pragma once

#ifdef ROS2
#define MAKE_SERVER(CallbackClass, ActionName, TopicName)                                                              \
  ensenso::std::make_unique<ActionName##Server>(                                                                       \
      nh, TopicName, ensenso::std::bind(&CallbackClass::on##ActionName, this, std::placeholders::_1));

#define MAKE_MONO_SERVER(CallbackClass, ActionName, TopicName)                                                         \
  ensenso::std::make_unique<ActionName##MonoServer>(                                                                   \
      nh, TopicName, ensenso::std::bind(&CallbackClass::on##ActionName, this, std::placeholders::_1));

#define USING_ENSENSO_CAMERA_ACTION(ACTION_NAME)                                                                       \
  namespace ensenso                                                                                                    \
  {                                                                                                                    \
  namespace action                                                                                                     \
  {                                                                                                                    \
  using ACTION_NAME##Action = ensenso_camera_msgs::action::ACTION_NAME;                                                \
  using ACTION_NAME##ActionPtr = ::std::shared_ptr<ACTION_NAME##Action>;                                               \
  using ACTION_NAME##ActionConstPtr = ::std::shared_ptr<ACTION_NAME##Action const>;                                    \
                                                                                                                       \
  using ACTION_NAME##Goal = typename ensenso_camera_msgs::action::ACTION_NAME::Goal;                                   \
  using ACTION_NAME##GoalPtr = ::std::shared_ptr<ACTION_NAME##Goal>;                                                   \
  using ACTION_NAME##GoalConstPtr = ::std::shared_ptr<ACTION_NAME##Goal const>;                                        \
                                                                                                                       \
  using ACTION_NAME##Result = typename ensenso_camera_msgs::action::ACTION_NAME::Result;                               \
  using ACTION_NAME##ResultPtr = ::std::shared_ptr<ACTION_NAME##Result>;                                               \
  using ACTION_NAME##ResultConstPtr = ::std::shared_ptr<ACTION_NAME##Result const>;                                    \
                                                                                                                       \
  using ACTION_NAME##Feedback = typename ensenso_camera_msgs::action::ACTION_NAME::Feedback;                           \
  using ACTION_NAME##FeedbackPtr = ::std::shared_ptr<ACTION_NAME##Feedback>;                                           \
  using ACTION_NAME##FeedbackConstPtr = ::std::shared_ptr<ACTION_NAME##Feedback const>;                                \
  }                                                                                                                    \
  }
#else
#define MAKE_SERVER(CallbackClass, ActionName, TopicName)                                                              \
  ensenso::std::make_unique<ActionName##Server>(nh, TopicName,                                                         \
                                                ensenso::std::bind(&CallbackClass::on##ActionName, this, _1));

#define MAKE_MONO_SERVER(CallbackClass, ActionName, TopicName)                                                         \
  ensenso::std::make_unique<ActionName##MonoServer>(nh, TopicName,                                                     \
                                                    ensenso::std::bind(&CallbackClass::on##ActionName, this, _1));

// In ROS1 the pointer typedefs look like this:
//  typedef boost::shared_ptr< ::ensenso_camera_msgs::AccessTreeAction_<ContainerAllocator> > Ptr;
//  typedef boost::shared_ptr< ::ensenso_camera_msgs::AccessTreeAction_<ContainerAllocator> const> ConstPtr;

#define USING_ENSENSO_CAMERA_ACTION(ACTION_NAME)                                                                       \
  namespace ensenso                                                                                                    \
  {                                                                                                                    \
  namespace action                                                                                                     \
  {                                                                                                                    \
  using ACTION_NAME##Action = ensenso_camera_msgs::ACTION_NAME##Action;                                                \
  using ACTION_NAME##ActionPtr = ensenso_camera_msgs::ACTION_NAME##ActionPtr;                                          \
  using ACTION_NAME##ActionConstPtr = ensenso_camera_msgs::ACTION_NAME##ActionConstPtr;                                \
                                                                                                                       \
  using ACTION_NAME##Goal = ensenso_camera_msgs::ACTION_NAME##Goal;                                                    \
  using ACTION_NAME##GoalPtr = ensenso_camera_msgs::ACTION_NAME##GoalPtr;                                              \
  using ACTION_NAME##GoalConstPtr = ensenso_camera_msgs::ACTION_NAME##GoalConstPtr;                                    \
                                                                                                                       \
  using ACTION_NAME##Result = ensenso_camera_msgs::ACTION_NAME##Result;                                                \
  using ACTION_NAME##ResultPtr = ensenso_camera_msgs::ACTION_NAME##ResultPtr;                                          \
  using ACTION_NAME##ResultConstPtr = ensenso_camera_msgs::ACTION_NAME##ResultConstPtr;                                \
                                                                                                                       \
  using ACTION_NAME##Feedback = ensenso_camera_msgs::ACTION_NAME##Feedback;                                            \
  using ACTION_NAME##FeedbackPtr = ensenso_camera_msgs::ACTION_NAME##FeedbackPtr;                                      \
  using ACTION_NAME##FeedbackConstPtr = ensenso_camera_msgs::ACTION_NAME##FeedbackConstPtr;                            \
  }                                                                                                                    \
  }
#endif

#define USING_ENSENSO_CAMERA_ACTION_WITH_SERVER(ACTION_NAME)                                                           \
  USING_ENSENSO_CAMERA_ACTION(ACTION_NAME)                                                                             \
  using ACTION_NAME##Server = QueuedActionServer<ensenso::action::ACTION_NAME##Action>;
