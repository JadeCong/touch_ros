#ifdef  _WIN64
#pragma warning (disable:4996)
#endif

#include <stdio.h>

#if defined(WIN32)
# include <windows.h>
# include <conio.h>
#else
# include "conio.h"
# include <stdlib.h>
# include <ctype.h>
# include <string.h>
#endif

#include <HD/hd.h>
#include <HDU/hduError.h>
#include <HDU/hduVector.h>

#include <iostream>
#include <vector>
#include <sstream>
#include <iterator>
#include <math.h>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/WrenchStamped.h>
#include <tf/transform_broadcaster.h>


/* Definition for ROS configuration. */
typedef struct
{
    /* ROS subscriber */
    ros::Subscriber sub_feedback_force;  // Subscriber for feedback force.
    
    hduVector3Dd m_FeedbackForce;  // Current feedback force to apply for device.
    hduVector3Dd m_FeedbackTorque;  // Current feedback torque to apply for device.
    
    std::string feedback_force_topic;  // Feedback force topic.
    
    /* ROS publisher */
    ros::Publisher pub_joint_states;  // Publisher for joint states.
    
    sensor_msgs::JointState joint_states;  // Joint states message.
    unsigned int command_index;  // Command index.
    HDdouble m_DeviceCurrentJointStates[6];  // Current joint state of device.
    HDdouble m_DeviceLastJointStates[6];  // Last joint state of device.
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
    ROS callback for synchronously getting the feedback force form hand.
*********************************************************************************************/
void GetFeedbackForceCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
    /* Feedback force from hand. */
    gROSConfig.m_FeedbackForce[0] = msg->wrench.force.x;  // unit: N
    gROSConfig.m_FeedbackForce[1] = msg->wrench.force.y;
    gROSConfig.m_FeedbackForce[2] = msg->wrench.force.z;
    
    /* Feedback torque from hand. */
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
    ros::init(argc, argv, "feedback_force_from_hand", ros::init_options::AnonymousName);
    ros::NodeHandle nh;
    
    /* Get the parameters form ros parameter server. */
    nh.getParam("/touch/feedback_force_from_hand/feedback_force_topic", gROSConfig.feedback_force_topic);
    
    /* Set the subscriber of feedback force. */
    gROSConfig.sub_feedback_force = nh.subscribe<geometry_msgs::WrenchStamped>(gROSConfig.feedback_force_topic, 1, GetFeedbackForceCallback, ros::TransportHints().reliable().tcpNoDelay());
    
    /* Set the publisher of device state and command. */
    gROSConfig.pub_joint_states = nh.advertise<sensor_msgs::JointState>("/touch/master_touch/joint_states", 1);
    
    printf("Configure ROS successfully.\n\n");
}

/*******************************************************************************
    Prints out a help string about using this node.
*******************************************************************************/
void PrintHelp(void)
{
    static const char help[] = {"\n\
Hold tight the stylus and press the button so you can feel the feedback force from hand in cartesian space.\n"};
    
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
        /* Render the feedback force and torque which comes from hand. */
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
            else
            {
                printf("\nInvalid keypress\n");
                printf("Press Q to quit or hold stylus tight.\n");
            }
        }
        
        /* Update the feedback force data from hand. */
        ros::spinOnce();
        
        /* Schedule the publish callback function for publishing the device state. */
        hdScheduleSynchronous(PublishDeviceStateCallback, 0, HD_DEFAULT_SCHEDULER_PRIORITY);
        
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


/******************************************************************************
    Main function.
    Sets up the device, runs main application loop, cleans up when finished.
******************************************************************************/
int main(int argc, char* argv[])
{
    /* Configure ros node. */
    printf("\nTouch_Teleoperation(Package): Feedback_Force_From_Hand(Node)\n\n");
    ConfigureROS(argc, argv);
    
    /* Initialize the device, must be done before attempting to call any hd functions. */
    HDErrorInfo error;
    HHD hHD = hdInitDevice(HD_DEFAULT_DEVICE);
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error, "Failed to initialize haptic device");
        fprintf(stderr, "\nPress any key to quit.\n");
        getch();
        return -1;
    }
    
    /* Start the haptic rendering loop. */
    hdStartScheduler();
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error, "Failed to start scheduler");
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
