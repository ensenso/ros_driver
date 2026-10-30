#pragma once

#ifdef ROS2 /**********************************************************************************************************/
#define USING_ENSENSO_CAMERA_MSG(MSG_NAME)                                                                             \
  namespace ensenso                                                                                                    \
  {                                                                                                                    \
  namespace msg                                                                                                        \
  {                                                                                                                    \
  using MSG_NAME = ensenso_camera_msgs::msg::MSG_NAME;                                                                 \
  using MSG_NAME##Ptr = ensenso_camera_msgs::msg::MSG_NAME::SharedPtr;                                                 \
  using MSG_NAME##ConstPtr = ensenso_camera_msgs::msg::MSG_NAME::ConstSharedPtr;                                       \
  }                                                                                                                    \
  }

#define USING_MSG(PACKAGE_NAME, MSG_NAME)                                                                              \
  namespace PACKAGE_NAME                                                                                               \
  {                                                                                                                    \
  namespace msg                                                                                                        \
  {                                                                                                                    \
  using MSG_NAME = PACKAGE_NAME::msg::MSG_NAME;                                                                        \
  using MSG_NAME##Ptr = PACKAGE_NAME::msg::MSG_NAME::SharedPtr;                                                        \
  using MSG_NAME##ConstPtr = PACKAGE_NAME::msg::MSG_NAME::ConstSharedPtr;                                              \
  }                                                                                                                    \
  }

#define GET_D_MATRIX(info) info->d
#define GET_K_MATRIX(info) info->k
#define GET_P_MATRIX(info) info->p
#define GET_R_MATRIX(info) info->r

#else /***ROS1*********************************************************************************************************/
#define USING_ENSENSO_CAMERA_MSG(MSG_NAME)                                                                             \
  namespace ensenso                                                                                                    \
  {                                                                                                                    \
  namespace msg                                                                                                        \
  {                                                                                                                    \
  using MSG_NAME = ensenso_camera_msgs::MSG_NAME;                                                                      \
  using MSG_NAME##Ptr = ensenso_camera_msgs::MSG_NAME##Ptr;                                                            \
  using MSG_NAME##ConstPtr = ensenso_camera_msgs::MSG_NAME##ConstPtr;                                                  \
  }                                                                                                                    \
  }

#define USING_MSG(PACKAGE_NAME, MSG_NAME)                                                                              \
  namespace PACKAGE_NAME                                                                                               \
  {                                                                                                                    \
  namespace msg                                                                                                        \
  {                                                                                                                    \
  using MSG_NAME = PACKAGE_NAME::MSG_NAME;                                                                             \
  using MSG_NAME##Ptr = PACKAGE_NAME::MSG_NAME##Ptr;                                                                   \
  using MSG_NAME##ConstPtr = PACKAGE_NAME::MSG_NAME##ConstPtr;                                                         \
  }                                                                                                                    \
  }

#define GET_D_MATRIX(info) info->D
#define GET_K_MATRIX(info) info->K
#define GET_P_MATRIX(info) info->P
#define GET_R_MATRIX(info) info->R
#endif
