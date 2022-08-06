#ifdef  _WIN64
#pragma warning (disable:4996)
#endif

#if defined(WIN32)
# include <windows.h>
# include <conio.h>
#else
# include "conio.h"
# include <string.h>
#endif

#include <unistd.h>
#include <time.h>
#include <stdio.h>
#include <assert.h>
#include <math.h>
#include <iostream>
#include <vector>
#include <sstream>
#include <iterator>

#include <HL/hl.h>
#include <HD/hd.h>
#include <HDU/hduVector.h>
#include <HDU/hduError.h>
#include <HDU/hduMatrix.h>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/WrenchStamped.h>
#include <tf/transform_broadcaster.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/GripperCommandAction.h>
#include <franka_gripper/HomingAction.h>
#include <franka_gripper/MoveAction.h>
#include <franka_gripper/StopAction.h>
#include <franka_gripper/GraspAction.h>
#include <franka_gripper/GraspAction.h>


/* Definition for ROS configuration. */
typedef struct
{
    /* ROS subscriber */
    ros::Subscriber sub_feedback_force;  // Subscriber for feedback force.
    
    hduVector3Dd m_FeedbackForce;  // Current feedback force to apply for device.
    hduVector3Dd m_FeedbackTorque;  // Current feedback torque to apply for device.
    
    std::string master_feedback_force_topic;  // Master feedback force topic.
    
    /* ROS publisher */
    ros::Publisher pub_joint_states;  // Publisher for joint states.
    ros::Publisher pub_cartesian_state;  // Publisher for cartesian state.
    ros::Publisher pub_cartesian_absolute_command;  // Publisher for cartesian absolute command.
    ros::Publisher pub_cartesian_incremental_command;  // Publisher for cartesian incremental command.
    
    sensor_msgs::JointState joint_states;  // Joint states message.
    geometry_msgs::PoseStamped cartesian_state;  // Cartesian state message.
    geometry_msgs::PoseStamped cartesian_absolute_command;  // Cartesian absolute command message.
    geometry_msgs::PoseStamped cartesian_incremental_command;  // Cartesian incremental command message.
    unsigned int command_index;  // Command index.
    HDdouble m_DeviceCurrentJointStates[6];  // Current joint state of device.
    HDdouble m_DeviceLastJointStates[6];  // Last joint state of device.
    hduVector3Dd m_DeviceInitialPositionCmd;  // Initial position command of device.
    hduVector3Dd m_DeviceInitialOrientationCmd;  // Initial orientation command of device. 
    
    std::string frame_cartesian_state;  // Cartesian state frame.
    std::string frame_cartesian_absolute_command;  // Cartesian absolute command frame.
    std::string frame_cartesian_incremental_command;  // Cartesian incremental command frame.
    
    /* ROS action client */
    actionlib::SimpleActionClient<franka_gripper::HomingAction>* gripper_homing_action_client;  // Franka gripper homing action client.
    actionlib::SimpleActionClient<franka_gripper::MoveAction>* gripper_move_action_client;  // Franka gripper move action client.
    actionlib::SimpleActionClient<franka_gripper::StopAction>* gripper_stop_action_client;  // Franka gripper stop action client.
    actionlib::SimpleActionClient<franka_gripper::GraspAction>* gripper_grasp_action_client;  // Franka gripper grasp action client.
    actionlib::SimpleActionClient<control_msgs::GripperCommandAction>* gripper_command_action_client;  // Franka gripper command action client.
    
    HDboolean button_pressed_continuous_flag;  // Flag for gimbal button pressed continuously.
    HDboolean button_pressed_count_flag;  // Flag for gimbal button pressed count.
    clock_t button_pressed_count_start_time;  // Start time for gimbal button pressed count.
    clock_t button_pressed_count_end_time;  // End time for gimbal button pressed count.
    HDint button_pressed_times;  // Touch gimbal black button pressed times.
    std::string gripper_action_goal_flag;  // Franka gripper action goal flag.
    
    franka_gripper::HomingGoal gripper_homing_goal;  // Franka gripper homing goal.
    franka_gripper::MoveGoal gripper_move_goal;  // Franka gripper move goal.
    franka_gripper::StopGoal gripper_stop_goal;  // Franka gripper stop goal.
    franka_gripper::GraspGoal gripper_grasp_goal;  // Franka gripper grasp goal.
    control_msgs::GripperCommandGoal gripper_command_goal;  // Franka gripper command goal.
    
    franka_gripper::HomingGoal touch_homing_goal;  // Franka gripper homing goal from touch.
    franka_gripper::MoveGoal touch_open_goal;  // Franka gripper open goal from touch.
    franka_gripper::MoveGoal touch_close_goal;  // Franka gripper close goal from touch.
    franka_gripper::GraspGoal touch_grasp_goal;  // Franka gripper grasp goal from touch.
    
    std::string gripper_homing_action_server;  // Franka gripper homing action server name.
    std::string gripper_move_action_server;  // Franka gripper move action server name.
    std::string gripper_stop_action_server;  // Franka gripper stop action server name.
    std::string gripper_grasp_action_server;  // Franka gripper grasp action server name.
    std::string gripper_command_action_server;  // Franka gripper command action server name.
} ROSConfig;

static ROSConfig gROSConfig = 
{
    /* ROS subscriber */
    .m_FeedbackForce = hduVector3Dd(0.0, 0.0, 0.0),
    .m_FeedbackTorque = hduVector3Dd(0.0, 0.0, 0.0),
    
    /* ROS publisher */
    .command_index = 0,
    .m_DeviceCurrentJointStates = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
    .m_DeviceLastJointStates = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
    .m_DeviceInitialPositionCmd = hduVector3Dd(0.0, 0.0, 0.0),
    .m_DeviceInitialOrientationCmd = hduVector3Dd(0.0, 0.0, 0.0),
    
    /* ROS action client */
    .button_pressed_continuous_flag = HD_FALSE,
    .button_pressed_count_flag = HD_FALSE,
    .button_pressed_count_start_time = 0,
    .button_pressed_count_end_time = 0,
    .button_pressed_times = 0,
    .gripper_action_goal_flag = "none",
    
    .gripper_homing_goal = franka_gripper::HomingGoal(),
    .gripper_move_goal = franka_gripper::MoveGoal(),
    .gripper_stop_goal = franka_gripper::StopGoal(),
    .gripper_grasp_goal = franka_gripper::GraspGoal(),
    .gripper_command_goal = control_msgs::GripperCommandGoal(),
    
    .touch_homing_goal = franka_gripper::HomingGoal(),
    .touch_open_goal = franka_gripper::MoveGoal(),
    .touch_close_goal = franka_gripper::MoveGoal(),
    .touch_grasp_goal = franka_gripper::GraspGoal(),
};

/* Definition for device data retrieved from HDAPI. */
typedef struct 
{
    /* Properties and Configurations State. */
    HDstring m_DeviceModelType;  // Device model type.
    HDdouble m_DeviceMaxWorkspaceDimensions[6];  // Maximum workspace dimensions of device.
    HDdouble m_DeviceUsableWorkspaceDimensions[6];  // Usable workspace dimensions of device.
    HDint m_DeviceInputDof;  // Input DOF of device.
    HDint m_DeviceOutputDof;  // Output DOF of device.
    HDint m_DeviceCalibrationStyle;  // Calibration style of device.
    
    HDint m_DeviceUpdateRate;  // Update rate of device(unit: Hz).
    HDint m_DeviceInstantaneousUpdateRate;  // Instantaneous update rate of device(unit: Hz).
    HDdouble m_DeviceNominalMaxStiffness;  // Nominal maximum force stiffness of device.
    HDdouble m_DeviceNominalMaxDamping;  // Nominal maximum force damping of device.
    HDdouble m_DeviceNominalMaxForce;  // Nominal maximum force of device(unit: N).
    HDdouble m_DeviceNominalMaxContinuousForce;  // Nominal maximum continuous force of device(unit: N).
    HDdouble m_DeviceForceRampingRate;  // Force ramping rate of device(unit: N/s).
    HDdouble m_DeviceNominalMaxTorqueStiffness;  // Nominal maximum torque stiffness of device.
    HDdouble m_DeviceNominalMaxTorqueDamping;  // Nominal maximum torque damping of device.
    HDdouble m_DeviceNominalMaxTorqueForce;  // Nominal maximum torque of device(unit: mNm).
    HDdouble m_DeviceNominalMaxTorqueContinuousForce;  // Nominal maximum continuous torque of device(unit: mNm).
    
    /* Current State(button/pose/force). */
    HDint m_DeviceCurrentButtons;  // Current buttons state of device.
    HDboolean m_DeviceCurrentButtonBlackState;  // Whether the current device black button(HD_DEVICE_BUTTON_1) is pressed.
    HDboolean m_DeviceCurrentButtonWhiteState;  // Whether the current device white button(HD_DEVICE_BUTTON_2) is pressed.
    
    hduVector3Dd m_DeviceCurrentPosition;  // Current hip position of device(unit: mm).
    hduVector3Dd m_DeviceCurrentVelocity;  // Current hip velocity of device(unit: mm/s).
    hduVector3Dd m_DeviceCurrentJointAngles;  // Current joint angles of device for computing the kinematics(unit: rad).
    hduVector3Dd m_DeviceCurrentGimbalAngles;  // Current gimbal angles of device(unit: rad).
    hduVector3Dd m_DeviceCurrentAngularVelocity;  // Current gimbal angular velocity of device(unit: rad/s).
    HDdouble m_DeviceCurrentTransform[16];  // Current hip transform of device.
    
    hduVector3Dd m_DeviceCurrentForce;  // Current query force of device in cartesian space(unit: N).
    hduVector3Dd m_DeviceCurrentTorque;  // Current query torque of device in cartesian space(unit: mNm).
    hduVector3Dd m_DeviceCurrentJointTorque;  // Current query joint torque of device in joint space(unit: mNm).
    hduVector3Dd m_DeviceCurrentGimbalTorque;  // Current query gimbal torque of device in joint space(unit: mNm).
    
    /* Last State(button/pose/force). */
    HDint m_DeviceLastButtons;   // Last buttons state of device.
    HDboolean m_DeviceLastButtonBlackState;  // Whether the last device black button(HD_DEVICE_BUTTON_1) is pressed.
    HDboolean m_DeviceLastButtonWhiteState;  // Whether the last device white button(HD_DEVICE_BUTTON_2) is pressed.
    
    hduVector3Dd m_DeviceLastPosition;  // Last hip position of device(unit: mm).
    hduVector3Dd m_DeviceLastVelocity;  // Last hip velocity of device(unit: mm/s).
    hduVector3Dd m_DeviceLastJointAngles;  // Last joint angles of device for for computing the kinematics(unit: rad).
    hduVector3Dd m_DeviceLastGimbalAngles;  // Last gimbal angles of device(unit: rad).
    hduVector3Dd m_DeviceLastAngularVelocity;  // Last gimbal angular velocity of device(unit: rad/s).
    HDdouble m_DeviceLastTransform[16];  // Last hip transform of device.
    
    hduVector3Dd m_DeviceLastForce;  // Last query force of device in cartesian space(unit: N).
    hduVector3Dd m_DeviceLastTorque;  // Last query torque of device in cartesian space(unit: mNm).
    hduVector3Dd m_DeviceLastJointTorque;  // Last query joint torque of device in joint space(unit: mNm).
    hduVector3Dd m_DeviceLastGimbalTorque;  // Last query gimbal torque of device in joint space(unit: mNm).
    
    /* Device Error. */
    HDErrorInfo m_DeviceError;  // Current HDAPI error state of device.
} DeviceData;

static DeviceData gServoDeviceData = 
{
    /* Current State(button/pose/force). */
    .m_DeviceCurrentButtons = 0,
    .m_DeviceCurrentButtonBlackState = HD_FALSE,
    .m_DeviceCurrentButtonWhiteState = HD_FALSE,
    
    .m_DeviceCurrentPosition = hduVector3Dd(0.0, 0.0, 0.0),
    .m_DeviceCurrentVelocity = hduVector3Dd(0.0, 0.0, 0.0),
    .m_DeviceCurrentJointAngles = hduVector3Dd(0.0, 0.0, 0.0),
    .m_DeviceCurrentGimbalAngles = hduVector3Dd(0.0, 0.0, 0.0),
    .m_DeviceCurrentAngularVelocity = hduVector3Dd(0.0, 0.0, 0.0),
    .m_DeviceCurrentTransform = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
    
    .m_DeviceCurrentForce = hduVector3Dd(0.0, 0.0, 0.0),
    .m_DeviceCurrentTorque = hduVector3Dd(0.0, 0.0, 0.0),
    .m_DeviceCurrentJointTorque = hduVector3Dd(0.0, 0.0, 0.0),
    .m_DeviceCurrentGimbalTorque = hduVector3Dd(0.0, 0.0, 0.0),
    
    /* Last State(button/pose/force). */
    .m_DeviceLastButtons = 0,
    .m_DeviceLastButtonBlackState = HD_FALSE,
    .m_DeviceLastButtonWhiteState = HD_FALSE,
    
    .m_DeviceLastPosition = hduVector3Dd(0.0, 0.0, 0.0),
    .m_DeviceLastVelocity = hduVector3Dd(0.0, 0.0, 0.0),
    .m_DeviceLastJointAngles = hduVector3Dd(0.0, 0.0, 0.0),
    .m_DeviceLastGimbalAngles = hduVector3Dd(0.0, 0.0, 0.0),
    .m_DeviceLastAngularVelocity = hduVector3Dd(0.0, 0.0, 0.0),
    .m_DeviceLastTransform = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
    
    .m_DeviceLastForce = hduVector3Dd(0.0, 0.0, 0.0),
    .m_DeviceLastTorque = hduVector3Dd(0.0, 0.0, 0.0),
    .m_DeviceLastJointTorque = hduVector3Dd(0.0, 0.0, 0.0),
    .m_DeviceLastGimbalTorque = hduVector3Dd(0.0, 0.0, 0.0),
};

HDSchedulerHandle gApplyFeedbackForceSchedulerHandle = HD_INVALID_HANDLE;
HDSchedulerHandle gQueryDeviceStateSchedulerHandle = HD_INVALID_HANDLE;


/*********************************************************************************************
    ROS callback for synchronously getting the feedback force form slave robot.
*********************************************************************************************/
void GetFeedbackForceCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
    /* Feedback force from slave robot. */
    gROSConfig.m_FeedbackForce[0] = msg->wrench.force.x;  // unit: N
    gROSConfig.m_FeedbackForce[1] = msg->wrench.force.y;
    gROSConfig.m_FeedbackForce[2] = msg->wrench.force.z;
    
    /* Feedback torque from slave robot. */
    gROSConfig.m_FeedbackTorque[0] = msg->wrench.torque.x * 1000.0;  // unit: Nm -> mNm
    gROSConfig.m_FeedbackTorque[1] = msg->wrench.torque.y * 1000.0;
    gROSConfig.m_FeedbackTorque[2] = msg->wrench.torque.z * 1000.0;
}

/****************************************
    Configure the ROS node.
*****************************************/
void ConfigureROS(int argc, char* argv[])
{
    /* Initialize ros node. */
    ros::init(argc, argv, "master_touch", ros::init_options::AnonymousName);
    ros::NodeHandle nh;
    
    /* Get the parameters form ros parameter server. */
    nh.getParam("/touch/master_touch/master_feedback_force_topic", gROSConfig.master_feedback_force_topic);
    
    nh.getParam("/touch/master_touch/frame_cartesian_state", gROSConfig.frame_cartesian_state);
    nh.getParam("/touch/master_touch/frame_cartesian_absolute_command", gROSConfig.frame_cartesian_absolute_command);
    nh.getParam("/touch/master_touch/frame_cartesian_incremental_command", gROSConfig.frame_cartesian_incremental_command);
    
    nh.getParam("/touch/master_touch/gripper_homing_action_server", gROSConfig.gripper_homing_action_server);
    nh.getParam("/touch/master_touch/gripper_move_action_server", gROSConfig.gripper_move_action_server);
    nh.getParam("/touch/master_touch/gripper_stop_action_server", gROSConfig.gripper_stop_action_server);
    nh.getParam("/touch/master_touch/gripper_grasp_action_server", gROSConfig.gripper_grasp_action_server);
    nh.getParam("/touch/master_touch/gripper_command_action_server", gROSConfig.gripper_command_action_server);
    
    nh.getParam("/touch/master_touch/touch_open_goal/width", gROSConfig.touch_open_goal.width);
    nh.getParam("/touch/master_touch/touch_open_goal/speed", gROSConfig.touch_open_goal.speed);
    nh.getParam("/touch/master_touch/touch_close_goal/width", gROSConfig.touch_close_goal.width);
    nh.getParam("/touch/master_touch/touch_close_goal/speed", gROSConfig.touch_close_goal.speed);
    nh.getParam("/touch/master_touch/touch_grasp_goal/width", gROSConfig.touch_grasp_goal.width);
    nh.getParam("/touch/master_touch/touch_grasp_goal/epsilon/inner", gROSConfig.touch_grasp_goal.epsilon.inner);
    nh.getParam("/touch/master_touch/touch_grasp_goal/epsilon/outer", gROSConfig.touch_grasp_goal.epsilon.outer);
    nh.getParam("/touch/master_touch/touch_grasp_goal/speed", gROSConfig.touch_grasp_goal.speed);
    nh.getParam("/touch/master_touch/touch_grasp_goal/force", gROSConfig.touch_grasp_goal.force);
    
    /* Set the subscriber of feedback force. */
    gROSConfig.sub_feedback_force = nh.subscribe<geometry_msgs::WrenchStamped>(gROSConfig.master_feedback_force_topic, 1, GetFeedbackForceCallback);
    
    /* Set the publisher of device state and command. */
    gROSConfig.pub_joint_states = nh.advertise<sensor_msgs::JointState>("/touch/master_touch/joint_states", 1);
    gROSConfig.pub_cartesian_state = nh.advertise<geometry_msgs::PoseStamped>("/touch/master_touch/cartesian_state", 1);
    gROSConfig.pub_cartesian_absolute_command = nh.advertise<geometry_msgs::PoseStamped>("/touch/master_touch/cartesian_absolute_command", 1);
    gROSConfig.pub_cartesian_incremental_command = nh.advertise<geometry_msgs::PoseStamped>("/touch/master_touch/cartesian_incremental_command", 1);
    
    /* Set the action client of gripper command. */
    gROSConfig.gripper_homing_action_client = new actionlib::SimpleActionClient<franka_gripper::HomingAction>(gROSConfig.gripper_homing_action_server, true);
    gROSConfig.gripper_move_action_client = new actionlib::SimpleActionClient<franka_gripper::MoveAction>(gROSConfig.gripper_move_action_server, true);
    gROSConfig.gripper_stop_action_client = new actionlib::SimpleActionClient<franka_gripper::StopAction>(gROSConfig.gripper_stop_action_server, true);
    gROSConfig.gripper_grasp_action_client = new actionlib::SimpleActionClient<franka_gripper::GraspAction>(gROSConfig.gripper_grasp_action_server, true);
    gROSConfig.gripper_command_action_client = new actionlib::SimpleActionClient<control_msgs::GripperCommandAction>(gROSConfig.gripper_command_action_server, true);
    
    /* Wait for action server started */
    gROSConfig.gripper_homing_action_client->waitForServer();
    gROSConfig.gripper_move_action_client->waitForServer();
    gROSConfig.gripper_stop_action_client->waitForServer();
    gROSConfig.gripper_grasp_action_client->waitForServer();
    gROSConfig.gripper_command_action_client->waitForServer();
    
    printf("Configure ROS successfully.\n\n");
}

/*******************************************************************************
    Prints out a help string about using this node.
*******************************************************************************/
void PrintHelp(void)
{
    static const char help[] = {"\n\
Hold tight the stylus and press the black button so you can feel the feedback force from slave robot in cartesian space.\n\
Press the stylus black button to publish the incremental pose of the device.\n\
Release the stylus black button to stop receiving feedback force and publishing incremental pose.\n\
Press the stylus white button once quickly(<=1500ms) to send gripper open action goal to slave hand action server.\n\
Press the stylus white button twice quickly(<=1500ms) to send gripper close action goal to slave hand action server.\n\
Press the stylus white button three times quickly(<=1500ms) to send gripper grasp action goal to slave hand action server.\n\
Press the stylus white button for a period time(=1500ms) to send gripper homing action goal to slave hand action server.\n\
The absolute pose of the device is always published until the application is terminated.\n\
Note: Press Q key to quit or press R key to reset the slave robot(Franka Panda/Gripper).\n"};

    fprintf(stdout, "%s\n", help);
}

/************************************************************
    Scheduler callback for configuring the Device(Touch).
*************************************************************/
HDCallbackCode HDCALLBACK ConfigureDeviceCallback(void *pUserData)
{
    HDErrorInfo error;
    
    hdBeginFrame(hdGetCurrentDevice());
    
    /* Get the properties of device. */
    gServoDeviceData.m_DeviceModelType = hdGetString(HD_DEVICE_MODEL_TYPE);
    hdGetDoublev(HD_MAX_WORKSPACE_DIMENSIONS, gServoDeviceData.m_DeviceMaxWorkspaceDimensions);
    hdGetDoublev(HD_USABLE_WORKSPACE_DIMENSIONS, gServoDeviceData.m_DeviceUsableWorkspaceDimensions);
    hdGetIntegerv(HD_INPUT_DOF, &gServoDeviceData.m_DeviceInputDof);
    hdGetIntegerv(HD_OUTPUT_DOF, &gServoDeviceData.m_DeviceOutputDof);
    hdGetIntegerv(HD_CALIBRATION_STYLE, &gServoDeviceData.m_DeviceCalibrationStyle);
    
    /* Get the configurations of device. */
    hdGetIntegerv(HD_UPDATE_RATE, &gServoDeviceData.m_DeviceUpdateRate);
    hdGetIntegerv(HD_INSTANTANEOUS_UPDATE_RATE, &gServoDeviceData.m_DeviceInstantaneousUpdateRate);
    hdGetDoublev(HD_NOMINAL_MAX_STIFFNESS, &gServoDeviceData.m_DeviceNominalMaxStiffness);
    hdGetDoublev(HD_NOMINAL_MAX_DAMPING, &gServoDeviceData.m_DeviceNominalMaxDamping);
    hdGetDoublev(HD_NOMINAL_MAX_FORCE, &gServoDeviceData.m_DeviceNominalMaxForce);
    hdGetDoublev(HD_NOMINAL_MAX_CONTINUOUS_FORCE, &gServoDeviceData.m_DeviceNominalMaxContinuousForce);
    hdGetDoublev(HD_FORCE_RAMPING_RATE, &gServoDeviceData.m_DeviceForceRampingRate);
    hdGetDoublev(HD_NOMINAL_MAX_TORQUE_STIFFNESS, &gServoDeviceData.m_DeviceNominalMaxTorqueStiffness);
    hdGetDoublev(HD_NOMINAL_MAX_TORQUE_DAMPING, &gServoDeviceData.m_DeviceNominalMaxTorqueDamping);
    hdGetDoublev(HD_NOMINAL_MAX_TORQUE_FORCE, &gServoDeviceData.m_DeviceNominalMaxTorqueForce);
    hdGetDoublev(HD_NOMINAL_MAX_TORQUE_CONTINUOUS_FORCE, &gServoDeviceData.m_DeviceNominalMaxTorqueContinuousForce);
    
    /* Set the scheduler rate for servo loop. */
    hdSetSchedulerRate(1000);  // could support 500/1000/1600Hz
    
    /* Enable/Disable capabilities of device. */
    hdEnable(HD_FORCE_OUTPUT);
    hdEnable(HD_MAX_FORCE_CLAMPING);
    hdEnable(HD_FORCE_RAMPING);
    
    hdEndFrame(hdGetCurrentDevice());
    
    /* Check if an error occurred while attempting to configure device. */
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        if (hduIsSchedulerError(&error))
        {
            hduPrintError(stderr, &error, "Failed to configure the device.\n");
            return HD_CALLBACK_DONE;
        }
    }
    else
    {
        printf("Configure Device successfully.\n\n");
    }
    
    return HD_CALLBACK_DONE;
}

/*****************************************************************
    Main scheduler callback for rendering the feedback force.
******************************************************************/
HDCallbackCode HDCALLBACK ApplyFeedbackForceCallback(void *pUserData)
{   
    HDErrorInfo error;
    static HDboolean bRenderForce = HD_FALSE;
    
    hdBeginFrame(hdGetCurrentDevice());
    
    /* Get the current/last button states. */
    hdGetIntegerv(HD_CURRENT_BUTTONS, &gServoDeviceData.m_DeviceCurrentButtons);
    hdGetIntegerv(HD_LAST_BUTTONS, &gServoDeviceData.m_DeviceLastButtons);
    
    if ((gServoDeviceData.m_DeviceCurrentButtons & HD_DEVICE_BUTTON_1) != 0 && 
        (gServoDeviceData.m_DeviceLastButtons & HD_DEVICE_BUTTON_1) == 0)
    {
        /* Detected button down. */
        bRenderForce = HD_TRUE;
    }
    else if ((gServoDeviceData.m_DeviceCurrentButtons & HD_DEVICE_BUTTON_1) == 0 && 
             (gServoDeviceData.m_DeviceLastButtons & HD_DEVICE_BUTTON_1) != 0)
    {
        /* Detected button up. */
        bRenderForce = HD_FALSE;
        
        /* Send zero force and torque to the device, or else it will just continue rendering the last force and torque sent. */
        hduVecSet(gROSConfig.m_FeedbackForce, 0.0, 0.0, 0.0);
        hdSetDoublev(HD_CURRENT_FORCE, gROSConfig.m_FeedbackForce);
        
        if (gServoDeviceData.m_DeviceOutputDof == 6)
        {
            hduVecSet(gROSConfig.m_FeedbackTorque, 0.0, 0.0, 0.0);
            hdSetDoublev(HD_CURRENT_TORQUE, gROSConfig.m_FeedbackTorque);
        }
    }
    
    /* Render feedback force/torque. */
    if (bRenderForce)
    {
        /* Render the feedback force and torque which comes from slave robot. */
        for (int i = 0; i < 3; i++)
        {
            if (gROSConfig.m_FeedbackForce[i] >= gServoDeviceData.m_DeviceNominalMaxForce)
            {
                gROSConfig.m_FeedbackForce[i] = gServoDeviceData.m_DeviceNominalMaxForce;
            }
            else if (gROSConfig.m_FeedbackForce[i] <= -gServoDeviceData.m_DeviceNominalMaxForce)
            {
                gROSConfig.m_FeedbackForce[i] = -gServoDeviceData.m_DeviceNominalMaxForce;
            }
        }
        hdSetDoublev(HD_CURRENT_FORCE, gROSConfig.m_FeedbackForce);
        
        if (gServoDeviceData.m_DeviceOutputDof == 6)
        {
            for (int j = 0; j < 3; j++)
            {
                if (gROSConfig.m_FeedbackTorque[j] >= gServoDeviceData.m_DeviceNominalMaxTorqueForce)
                {
                    gROSConfig.m_FeedbackTorque[j] = gServoDeviceData.m_DeviceNominalMaxTorqueForce;
                }
                else if (gROSConfig.m_FeedbackTorque[j] <= -gServoDeviceData.m_DeviceNominalMaxTorqueForce)
                {
                    gROSConfig.m_FeedbackTorque[j] = -gServoDeviceData.m_DeviceNominalMaxTorqueForce;
                }
            }
            hdSetDoublev(HD_CURRENT_TORQUE, gROSConfig.m_FeedbackTorque);
        }
    }
    
    hdEndFrame(hdGetCurrentDevice());
    
    /* Check if an error occurred while attempting to render the feedback force. */
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        if (hduIsForceError(&error))
        {
            bRenderForce = HD_FALSE;
        }
        else if (hduIsSchedulerError(&error))
        {
            hduPrintError(stderr, &error, "Error detected while rendering the feedback force.\n");
            return HD_CALLBACK_DONE;
        }
    }
    
    /* Signify that the callback should continue running, i.e. that it will be called again the next scheduler tick. */
    return HD_CALLBACK_CONTINUE;
}

/**************************************************************************************************************************
    Scheduler callback for querying haptic device state: button, position, orientation, joint state, force and torque.
**************************************************************************************************************************/
HDCallbackCode HDCALLBACK QueryDeviceStateCallback(void *pUserData)
{
    hdBeginFrame(hdGetCurrentDevice());
    
    /* Get the current/last button states. */
    hdGetIntegerv(HD_CURRENT_BUTTONS, &gServoDeviceData.m_DeviceCurrentButtons);
    hdGetIntegerv(HD_LAST_BUTTONS, &gServoDeviceData.m_DeviceLastButtons);
    
    /* Get the black and white gimbal button pressed state(boolean) for current/last time. */
    gServoDeviceData.m_DeviceCurrentButtonBlackState = (gServoDeviceData.m_DeviceCurrentButtons & HD_DEVICE_BUTTON_1) ? HD_TRUE : HD_FALSE;
    gServoDeviceData.m_DeviceCurrentButtonWhiteState = (gServoDeviceData.m_DeviceCurrentButtons & HD_DEVICE_BUTTON_2) ? HD_TRUE : HD_FALSE;
    gServoDeviceData.m_DeviceLastButtonBlackState = (gServoDeviceData.m_DeviceLastButtons & HD_DEVICE_BUTTON_1) ? HD_TRUE : HD_FALSE;
    gServoDeviceData.m_DeviceLastButtonWhiteState = (gServoDeviceData.m_DeviceLastButtons & HD_DEVICE_BUTTON_2) ? HD_TRUE : HD_FALSE;
    
    /*  Get the current hip position and current gimbal angles of device. */
    hdGetDoublev(HD_CURRENT_POSITION, gServoDeviceData.m_DeviceCurrentPosition);
    hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, gServoDeviceData.m_DeviceCurrentGimbalAngles);
    
    /* Get the current joint state of device. */
    hdGetDoublev(HD_CURRENT_JOINT_ANGLES, gServoDeviceData.m_DeviceCurrentJointAngles);
    float temp_joint_states[7] = {
        0.0,
        gServoDeviceData.m_DeviceCurrentJointAngles[0],
        gServoDeviceData.m_DeviceCurrentJointAngles[1],
        gServoDeviceData.m_DeviceCurrentJointAngles[2] - gServoDeviceData.m_DeviceCurrentJointAngles[1],
        gServoDeviceData.m_DeviceCurrentGimbalAngles[0],
        gServoDeviceData.m_DeviceCurrentGimbalAngles[1],
        gServoDeviceData.m_DeviceCurrentGimbalAngles[2],
    };
    gROSConfig.m_DeviceCurrentJointStates[0] = -temp_joint_states[1];
    gROSConfig.m_DeviceCurrentJointStates[1] = temp_joint_states[2];
    gROSConfig.m_DeviceCurrentJointStates[2] = temp_joint_states[3];
    gROSConfig.m_DeviceCurrentJointStates[3] = -temp_joint_states[4] + M_PI;
    gROSConfig.m_DeviceCurrentJointStates[4] = -temp_joint_states[5] - 3*M_PI/4;
    gROSConfig.m_DeviceCurrentJointStates[5] = temp_joint_states[6] - M_PI;
    
    /* Get the current query force and torque of device. */
    hdGetDoublev(HD_CURRENT_FORCE, gServoDeviceData.m_DeviceCurrentForce);
    hdGetDoublev(HD_CURRENT_TORQUE, gServoDeviceData.m_DeviceCurrentTorque);
    
    /* Also get the error state of HDAPI. */
    gServoDeviceData.m_DeviceError = hdGetError();
    
    hdEndFrame(hdGetCurrentDevice());
    
    /* Configure the action goal flag for franka gripper. */
    if (!gROSConfig.button_pressed_count_flag)
    {
        /* Set the button_pressed_count_flag if the button pressed for the first time. */
        if (gServoDeviceData.m_DeviceCurrentButtonWhiteState && !gServoDeviceData.m_DeviceLastButtonWhiteState)
        {
            gROSConfig.button_pressed_count_flag = HD_TRUE;
            gROSConfig.button_pressed_continuous_flag = HD_TRUE;
            gROSConfig.button_pressed_count_start_time = clock();
        }
    }
    if (gROSConfig.button_pressed_count_flag)
    {
        /* Check whether the gimbal button released for button_pressed_continuous_flag. */
        if (!gServoDeviceData.m_DeviceCurrentButtonWhiteState)
        {
            gROSConfig.button_pressed_continuous_flag = HD_FALSE;
        }
        /* Record the gimbal button pressed times. */
        if (gServoDeviceData.m_DeviceCurrentButtonWhiteState && !gServoDeviceData.m_DeviceLastButtonWhiteState)
        {
            gROSConfig.button_pressed_times += 1;
        }
    }
    
    /* Check if an error occurred while attempting to query the device state. */
    if (HD_DEVICE_ERROR(gServoDeviceData.m_DeviceError))
    {
        if (hduIsSchedulerError(&gServoDeviceData.m_DeviceError))
        {
            hduPrintError(stderr, &gServoDeviceData.m_DeviceError, "Error detected while querying the device state.\n");
            return HD_CALLBACK_DONE;
        }
    }
    
    /* Signify that the callback should continue running, i.e. that it will be called again the next scheduler tick. */
    return HD_CALLBACK_CONTINUE;
}

/********************************************************************
    Main scheduler callback for publishing the device state.
*********************************************************************/
HDCallbackCode HDCALLBACK PublishDeviceStateCallback(void *pUserData)
{
    /* Set the Touch joint states and publish the msgs. */
    gROSConfig.joint_states.header.stamp = ros::Time::now();
    gROSConfig.joint_states.name.resize(6);
    gROSConfig.joint_states.name[0] = "waist";
    gROSConfig.joint_states.name[1] = "shoulder";
    gROSConfig.joint_states.name[2] = "elbow";
    gROSConfig.joint_states.name[3] = "yaw";
    gROSConfig.joint_states.name[4] = "pitch";
    gROSConfig.joint_states.name[5] = "roll";
    gROSConfig.joint_states.position.resize(6);
    for (int i = 0; i < 6; i++)
    {
        gROSConfig.joint_states.position[i] = gROSConfig.m_DeviceCurrentJointStates[i];
    }
    gROSConfig.pub_joint_states.publish(gROSConfig.joint_states);
    
    /* Set the Touch cartesian state and publish the msgs. */
    gROSConfig.cartesian_state.header.stamp = ros::Time::now();
    gROSConfig.cartesian_state.header.frame_id = gROSConfig.frame_cartesian_state;
    gROSConfig.cartesian_state.pose.position.x = gServoDeviceData.m_DeviceCurrentPosition[0] / 1000.0;  // unit: mm/1000 -> m
    gROSConfig.cartesian_state.pose.position.y = gServoDeviceData.m_DeviceCurrentPosition[1] / 1000.0;
    gROSConfig.cartesian_state.pose.position.z = gServoDeviceData.m_DeviceCurrentPosition[2] / 1000.0;
    gROSConfig.cartesian_state.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(
                                                    gServoDeviceData.m_DeviceCurrentGimbalAngles[2], 
                                                    gServoDeviceData.m_DeviceCurrentGimbalAngles[1], 
                                                    gServoDeviceData.m_DeviceCurrentGimbalAngles[0]);
    gROSConfig.pub_cartesian_state.publish(gROSConfig.cartesian_state);
    
    /* Set the Touch cartesian absolute command and publish the msgs. */
    gROSConfig.cartesian_absolute_command.header.stamp = ros::Time::now();
    gROSConfig.cartesian_absolute_command.header.frame_id = gROSConfig.frame_cartesian_absolute_command;
    gROSConfig.cartesian_absolute_command.pose.position.x = gServoDeviceData.m_DeviceCurrentPosition[0] / 1000.0;  // unit: mm/1000 -> m
    gROSConfig.cartesian_absolute_command.pose.position.y = gServoDeviceData.m_DeviceCurrentPosition[1] / 1000.0;
    gROSConfig.cartesian_absolute_command.pose.position.z = gServoDeviceData.m_DeviceCurrentPosition[2] / 1000.0;
    gROSConfig.cartesian_absolute_command.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(
                                                                gServoDeviceData.m_DeviceCurrentGimbalAngles[2], 
                                                                gServoDeviceData.m_DeviceCurrentGimbalAngles[1], 
                                                                gServoDeviceData.m_DeviceCurrentGimbalAngles[0]);
    gROSConfig.pub_cartesian_absolute_command.publish(gROSConfig.cartesian_absolute_command);
    
    /* Set the Touch cartesian incremental command and publish the msgs. If the user depresses the gimbal black button, 
       calculate the incremental displacement(currentPose - initialPose), and send the displacement to slave robot for position control. */
    if ((gServoDeviceData.m_DeviceCurrentButtons & HD_DEVICE_BUTTON_1) == 0)  // button released
    {
        /* Set the Touch cartesian incremental command. */
        gROSConfig.command_index = 0;
        
        gROSConfig.cartesian_incremental_command.header.stamp = ros::Time::now();
        gROSConfig.cartesian_incremental_command.header.frame_id = gROSConfig.frame_cartesian_incremental_command;
        gROSConfig.cartesian_incremental_command.pose.position.x = 0.0;  // unit: mm/1000 -> m
        gROSConfig.cartesian_incremental_command.pose.position.y = 0.0;
        gROSConfig.cartesian_incremental_command.pose.position.z = 0.0;
        gROSConfig.cartesian_incremental_command.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, 0.0);
    }
    else if ((gServoDeviceData.m_DeviceCurrentButtons & HD_DEVICE_BUTTON_1) != 0 && 
             (gServoDeviceData.m_DeviceLastButtons & HD_DEVICE_BUTTON_1) == 0)  // button pressed for the first time
    {
        /* Set the Touch cartesian incremental command. */
        gROSConfig.m_DeviceInitialPositionCmd = gServoDeviceData.m_DeviceCurrentPosition;  // set the m_DeviceInitialPositionCmd for Touch cartesian incremental command
        gROSConfig.m_DeviceInitialOrientationCmd = gServoDeviceData.m_DeviceCurrentGimbalAngles;  // set the m_DeviceInitialOrientationCmd for Touch cartesian incremental command
        
        gROSConfig.cartesian_incremental_command.header.stamp = ros::Time::now();
        gROSConfig.cartesian_incremental_command.header.frame_id = gROSConfig.frame_cartesian_incremental_command;
        gROSConfig.cartesian_incremental_command.pose.position.x = 0.0;  // unit: mm/1000 -> m
        gROSConfig.cartesian_incremental_command.pose.position.y = 0.0;
        gROSConfig.cartesian_incremental_command.pose.position.z = 0.0;
        gROSConfig.cartesian_incremental_command.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, 0.0);
    }
    else if ((gServoDeviceData.m_DeviceCurrentButtons & HD_DEVICE_BUTTON_1) != 0 && 
             (gServoDeviceData.m_DeviceLastButtons & HD_DEVICE_BUTTON_1) != 0)  // button pressed beyond the first time
    {
        /* Set the Touch cartesian incremental command. */
        gROSConfig.command_index += 1;
        
        gROSConfig.cartesian_incremental_command.header.stamp = ros::Time::now();
        gROSConfig.cartesian_incremental_command.header.frame_id = gROSConfig.frame_cartesian_incremental_command;
        gROSConfig.cartesian_incremental_command.pose.position.x = (gServoDeviceData.m_DeviceCurrentPosition[0] - gROSConfig.m_DeviceInitialPositionCmd[0]) / 1000.0;  // unit: mm/1000 -> m
        gROSConfig.cartesian_incremental_command.pose.position.y = (gServoDeviceData.m_DeviceCurrentPosition[1] - gROSConfig.m_DeviceInitialPositionCmd[1]) / 1000.0;
        gROSConfig.cartesian_incremental_command.pose.position.z = (gServoDeviceData.m_DeviceCurrentPosition[2] - gROSConfig.m_DeviceInitialPositionCmd[2]) / 1000.0;
        gROSConfig.cartesian_incremental_command.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(
                                                                        gServoDeviceData.m_DeviceCurrentGimbalAngles[2] - gROSConfig.m_DeviceInitialOrientationCmd[2], 
                                                                        gServoDeviceData.m_DeviceCurrentGimbalAngles[1] - gROSConfig.m_DeviceInitialOrientationCmd[1], 
                                                                        gServoDeviceData.m_DeviceCurrentGimbalAngles[0] - gROSConfig.m_DeviceInitialOrientationCmd[0]);
    }
    gROSConfig.pub_cartesian_incremental_command.publish(gROSConfig.cartesian_incremental_command);
    
    /* Check if an error occurred while attempting to publish the device state. */
    if (HD_DEVICE_ERROR(gServoDeviceData.m_DeviceError))
    {
        if (hduIsSchedulerError(&gServoDeviceData.m_DeviceError))
        {
            hduPrintError(stderr, &gServoDeviceData.m_DeviceError, "Error detected while publishing the device state.\n");
            return HD_CALLBACK_DONE;
        }
    }
    
    return HD_CALLBACK_DONE;
}

/*********************************************************************************
    Main function callback for configuring the franka gripper action goal flag.
**********************************************************************************/
void ConfigureActionGoalFlagCallback(void *pUserData)
{
    /* Check whether the gimbal button pressed cycle(1500ms) complete. */
    gROSConfig.button_pressed_count_end_time = clock();
    
    /* TODO: Bugs: A time cycle(1500ms) is enough to press the gimbal white button for three times, but pressing 
            the button continuously over a cycle period will trigger the next cycle count while sending HomingGoal. 
            So, how to solve the bug? */
    if (double(gROSConfig.button_pressed_count_end_time - gROSConfig.button_pressed_count_start_time) / CLOCKS_PER_SEC >= 1.5)
    {
        /* Reset the gimbal button pressed count flag. */
        gROSConfig.button_pressed_count_flag = HD_FALSE;
        gROSConfig.button_pressed_count_start_time = 0;
        gROSConfig.button_pressed_count_end_time = 0;
        
        /* Configure the gripper action goal flag. */
        if (gROSConfig.button_pressed_continuous_flag)
        {
            gROSConfig.gripper_action_goal_flag = "homing";
        }
        else
        {
            if (gROSConfig.button_pressed_times == 0)
            {
                gROSConfig.gripper_action_goal_flag = "none";
            }
            else if (gROSConfig.button_pressed_times == 1)
            {
                gROSConfig.gripper_action_goal_flag = "open";
            }
            else if (gROSConfig.button_pressed_times == 2)
            {
                gROSConfig.gripper_action_goal_flag = "close";
            }
            else if (gROSConfig.button_pressed_times == 3)
            {
                gROSConfig.gripper_action_goal_flag = "grasp";
            }
        }
        
        /* Reset the gimbal button pressed times and continuous flag. */
        gROSConfig.button_pressed_times = 0;
        gROSConfig.button_pressed_continuous_flag = HD_FALSE;
    }
}

/*************************************************************************
    Main scheduler callback for sending the action goal to action server.
**************************************************************************/
HDCallbackCode HDCALLBACK SendActionGoalCallback(void *pUserData)
{
    /* Configure the franka gripper action goal flag if the button_pressed_count_flag will set true in a cycle(1500ms). */
    if (gROSConfig.button_pressed_count_flag)
    {
        ConfigureActionGoalFlagCallback(0);
    }
    
    /* Check the gripper action goal flag and decide which action goal to send to action server. */
    if (gROSConfig.gripper_action_goal_flag != "none")
    {
        if (gROSConfig.gripper_action_goal_flag == "open")
        {
            gROSConfig.gripper_move_action_client->sendGoal(gROSConfig.touch_open_goal);
            printf("Master_Touch: Open the franka gripper and wait for the process to complete...\n\n");
        }
        else if (gROSConfig.gripper_action_goal_flag == "close")
        {
            gROSConfig.gripper_move_action_client->sendGoal(gROSConfig.touch_close_goal);
            printf("Master_Touch: Close the franka gripper and wait for the process to complete...\n\n");
        }
        else if (gROSConfig.gripper_action_goal_flag == "grasp")
        {
            gROSConfig.gripper_grasp_action_client->sendGoal(gROSConfig.touch_grasp_goal);
            printf("Master_Touch: Grasp the target object and wait for the process to complete...\n\n");
        }
        else if (gROSConfig.gripper_action_goal_flag == "homing")
        {
            gROSConfig.gripper_homing_action_client->sendGoal(gROSConfig.touch_homing_goal);
            printf("Master_Touch: Homing the franka gripper and wait for the process to complete...\n\n");
        }
        
        /* Reset the gripper action goal flag once the action goal has been sent to action server. */
        gROSConfig.gripper_action_goal_flag = "none";
    }
    
    /* Check if an error occurred while attempting to send the action goal. */
    if (HD_DEVICE_ERROR(gServoDeviceData.m_DeviceError))
    {
        if (hduIsSchedulerError(&gServoDeviceData.m_DeviceError))
        {
            hduPrintError(stderr, &gServoDeviceData.m_DeviceError, "Error detected while sending the action goal.\n");
            return HD_CALLBACK_DONE;
        }
    }
    
    return HD_CALLBACK_DONE;
}

/*******************************************************************
    Function for reseting the slave robot(Franka Panda/Gripper).
********************************************************************/
void ResetSlaveRobot(void)
{
    /* Set the flag for reseting slave robot. */
    gROSConfig.cartesian_incremental_command.header.stamp = ros::Time::now();
    gROSConfig.cartesian_incremental_command.header.frame_id = gROSConfig.frame_cartesian_incremental_command;
    gROSConfig.cartesian_incremental_command.pose.position.x = -1.0;  // -1.0 is a flag number for resetting the slave robot.
    gROSConfig.cartesian_incremental_command.pose.position.y = -1.0;
    gROSConfig.cartesian_incremental_command.pose.position.z = -1.0;
    gROSConfig.cartesian_incremental_command.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, 0.0);
    
    /* Send the reset flag to slave robot. */
    gROSConfig.gripper_homing_action_client->sendGoal(gROSConfig.touch_homing_goal);
    for (int i = 0; i < 5; i++)  // publish reset flag for 5 times so that slave robot could receive the flag.
    {
        gROSConfig.pub_cartesian_incremental_command.publish(gROSConfig.cartesian_incremental_command);
    }
    printf("Resetting slave robot(Franka Panda/Gripper) and do not do any operation...\n\n");
    
    /* Wait 10 seconds for reseting slave robot and recover to command mode. */
    sleep(10);  // 10 seconds are enough for reseting slave robot.
    printf("Reset slave robot done and recovering to command mode...\n\n");
}

/********************************************
    Main loop for user application.
*********************************************/
void MainLoop(void)
{
    /* Definition for keypress flag. */
    int keypress;
    
    /* Run the main loop until the quit key pressed. */
    while (HD_TRUE)
    {
        /* Check for keyboard input. */
        if (_kbhit())
        {
            keypress = getch();
            keypress = toupper(keypress);
            
            if (keypress == 'Q')
            {
                return;
            }
            else if (keypress == 'R')
            {
                ResetSlaveRobot();
            }
            else
            {
                printf("\nInvalid keypress.\n");
                printf("Press Q key to quit or press R key to reset the slave robot(Franka Panda/Gripper).\n");
            }
        }
        
        /* Update the feedback force data from slave robot. */
        ros::spinOnce();
        
        /* Schedule the publish callback function for publishing the device state. */
        hdScheduleSynchronous(PublishDeviceStateCallback, 0, HD_DEFAULT_SCHEDULER_PRIORITY);
        
        /* Schedule the action callback function for sending goal to action server. */
        hdScheduleSynchronous(SendActionGoalCallback, 0, HD_DEFAULT_SCHEDULER_PRIORITY);
        
        /* Check if the main scheduler callback has exited. */
        if (!hdWaitForCompletion(gApplyFeedbackForceSchedulerHandle, HD_WAIT_CHECK_STATUS) || 
            !hdWaitForCompletion(gQueryDeviceStateSchedulerHandle, HD_WAIT_CHECK_STATUS))
        {
            fprintf(stderr, "\nThe main scheduler callback has exited\n");
            fprintf(stderr, "\nPress any key to quit.\n");
            getch();
            return;
        }
    }
}


/*******************************************************************************
    Main function.
    Sets up the device, runs main application loop, cleans up when finished.
*******************************************************************************/
int main(int argc, char* argv[])
{
    /* Configure ros node. */
    printf("\nTouch_Teleoperation(Package): Master_Touch(Node)\n\n");
    ConfigureROS(argc, argv);
    
    /* Initialize the device, must be done before attempting to call any hd functions. */
    HDErrorInfo error;
    HHD hHD = hdInitDevice(HD_DEFAULT_DEVICE);
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error, "Failed to initialize the device.");
        fprintf(stderr, "\nPress any key to quit.\n");
        getch();
        return -1;
    }
    
    /* Start the haptic rendering loop. */
    hdStartScheduler();
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error, "Failed to start the scheduler.");
        fprintf(stderr, "\nPress any key to quit.\n");
        getch();
        return -1;
    }
    
    /* Configure the device Touch. */
    hdScheduleSynchronous(ConfigureDeviceCallback, 0, HD_DEFAULT_SCHEDULER_PRIORITY);
    printf("Device Model Type: %s\n", gServoDeviceData.m_DeviceModelType);
    printf("Device Input DOF: %d DOF\n", gServoDeviceData.m_DeviceInputDof);
    printf("Device Output DOF: %d DOF\n", gServoDeviceData.m_DeviceOutputDof);
    printf("Device Update Rate: %d Hz\n", gServoDeviceData.m_DeviceUpdateRate);
    printf("Device Nominal Max Force: %.6f N\n", gServoDeviceData.m_DeviceNominalMaxForce);
    printf("Device Nominal Max Continuous Force: %.6f N\n", gServoDeviceData.m_DeviceNominalMaxContinuousForce);
    printf("Device Nominal Max Torque: %.6f mNm\n", gServoDeviceData.m_DeviceNominalMaxTorqueForce);
    printf("Device Nominal Max Continuous Torque: %.6f mNm\n", gServoDeviceData.m_DeviceNominalMaxTorqueContinuousForce);
    
    /* Schedule the haptic callback function for continuously rendering the feedback force. */
    gApplyFeedbackForceSchedulerHandle = hdScheduleAsynchronous(ApplyFeedbackForceCallback, 0, HD_MAX_SCHEDULER_PRIORITY);
    
    /* Schedule the query callback function for continuously querying the device state. */
    gQueryDeviceStateSchedulerHandle = hdScheduleAsynchronous(QueryDeviceStateCallback, 0, HD_DEFAULT_SCHEDULER_PRIORITY);
    
    /* Print out the help infos about this node. */
    PrintHelp();
    
    /* Start the main loop. */
    MainLoop();
    
    /* Cleanup by stopping the haptic loop, unscheduling the asynchronous callback, disabling the device. */
    hdStopScheduler();
    hdUnschedule(gApplyFeedbackForceSchedulerHandle);
    hdUnschedule(gQueryDeviceStateSchedulerHandle);
    hdDisableDevice(hHD);
    
    return 0;
}
