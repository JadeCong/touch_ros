/*****************************************************************************

Copyright (c) 2004 SensAble Technologies, Inc. All rights reserved.

OpenHaptics(TM) toolkit. The material embodied in this software and use of
this software is subject to the terms and conditions of the clickthrough
Development License Agreement.

For questions, comments or bug reports, go to forums at: 
    http://dsc.sensable.com

Module Name:
  
  touch_query_state.cpp

Description:

  This example demonstrates how to retrieve information from the haptic device.

*******************************************************************************/
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
#include <stdio.h>
#include <assert.h>

#include <HD/hd.h>
#include <HDU/hduVector.h>
#include <HDU/hduError.h>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>


/* Definition for ROS configuration. */
typedef struct
{
    /* ROS publisher */
    ros::Publisher pub_joint_states;  // Publisher for joint states.
    sensor_msgs::JointState joint_states;  // Joint states message.
    HDdouble m_DeviceCurrentJointStates[6];  // Current joint state of device.
    HDdouble m_DeviceLastJointStates[6];  // Last joint state of device.
    unsigned int command_index;  // Command index.
    hduVector3Dd m_PositionCommand;  // Position command.
    hduVector3Dd m_OrientationCommand;  // Orientation command.
    hduVector3Dd m_InitialPositionCommand;  // Initial position command.
    hduVector3Dd m_InitialOrientationCommand;  // Initial orientation command.
    HDboolean m_IncrementalCommandFlag;  // Incremental command flag.
} ROSConfig;

static ROSConfig gROSConfig = 
{
    .m_DeviceCurrentJointStates = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
    .m_DeviceLastJointStates = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
    .command_index = 0,
    .m_PositionCommand = hduVector3Dd(0.0, 0.0, 0.0),
    .m_OrientationCommand = hduVector3Dd(0.0, 0.0, 0.0),
    .m_InitialPositionCommand = hduVector3Dd(0.0, 0.0, 0.0),
    .m_InitialOrientationCommand = hduVector3Dd(0.0, 0.0, 0.0),
    .m_IncrementalCommandFlag = HD_FALSE,
};

/* Holds data retrieved from HDAPI. */
typedef struct 
{
    /* Current State(button/position/orientation). */
    HDint m_DeviceCurrentButtons;  // Current buttons state of device.
    HDboolean m_DeviceCurrentButtonState;  // Whether the current device black button(HD_DEVICE_BUTTON_1) is pressed.
    hduVector3Dd m_DeviceCurrentPosition;  // Current hip position of device(unit: mm).
    hduVector3Dd m_DeviceCurrentJointAngles;  // Current joint angles of device for computing the kinematics(unit: rad).
    hduVector3Dd m_DeviceCurrentGimbalAngles;  // Current gimbal angles of device(unit: rad).
    /* Last State(button/position/orientation). */
    HDint m_DeviceLastButtons;   // Last buttons state of device.
    HDboolean m_DeviceLastButtonState;  // Whether the last device black button(HD_DEVICE_BUTTON_1) is pressed.
    hduVector3Dd m_DeviceLastPosition;  // Last hip position of device(unit: mm).
    hduVector3Dd m_DeviceLastJointAngles;  // Last joint angles of device for for computing the kinematics(unit: rad).
    hduVector3Dd m_DeviceLastGimbalAngles;  // Last gimbal angles of device(unit: rad).
    /* Device Error. */
    HDErrorInfo m_DeviceError;  // Current HDAPI error state of device.
} DeviceData;

static DeviceData gServoDeviceData = 
{
    /* Current State(button/position/orientation). */
    .m_DeviceCurrentButtons = 0,
    .m_DeviceCurrentButtonState = HD_FALSE,
    .m_DeviceCurrentPosition = hduVector3Dd(0.0, 0.0, 0.0),
    .m_DeviceCurrentJointAngles = hduVector3Dd(0.0, 0.0, 0.0),
    .m_DeviceCurrentGimbalAngles = hduVector3Dd(0.0, 0.0, 0.0),
    /* Last State(button/position/orientation). */
    .m_DeviceLastButtons = 0,
    .m_DeviceLastButtonState = HD_FALSE,
    .m_DeviceLastPosition = hduVector3Dd(0.0, 0.0, 0.0),
    .m_DeviceLastJointAngles = hduVector3Dd(0.0, 0.0, 0.0),
    .m_DeviceLastGimbalAngles = hduVector3Dd(0.0, 0.0, 0.0),
};

HDSchedulerHandle gQueryDeviceStateSchedulerHandle = HD_INVALID_HANDLE;


/****************************************
    Configure the ROS node.
*****************************************/
void ConfigureROS(int argc, char* argv[])
{
    /* Initialize ros node. */
    ros::init(argc, argv, "touch_query_state", ros::init_options::AnonymousName);
    ros::NodeHandle nh;
    
    /* Set the publisher of device state. */
    gROSConfig.pub_joint_states = nh.advertise<sensor_msgs::JointState>("/touch/joint_states", 1);
    
    printf("Configure ROS successfully.\n\n");
}

/*******************************************************************************
    Prints out a help string about using this node.
*******************************************************************************/
void PrintHelp(void)
{
    static const char help[] = {"\n\
Hold tight the stylus and update the touch state to rviz.\n\
Press and release the stylus button to print out the current device location.\n\
Press Q/q key to exit the application.\n"};

    fprintf(stdout, "%s\n", help);
}

/*******************************************************************************
    Checks the state of the gimbal button and gets the position of the device.
*******************************************************************************/
HDCallbackCode HDCALLBACK QueryDeviceStateCallback(void *pUserData)
{
    hdBeginFrame(hdGetCurrentDevice());
    
    /* Get the current/last button states. */
    hdGetIntegerv(HD_CURRENT_BUTTONS, &gServoDeviceData.m_DeviceCurrentButtons);
    hdGetIntegerv(HD_LAST_BUTTONS, &gServoDeviceData.m_DeviceLastButtons);
    
    /* Get the black gimbal button pressed state(boolean) for current/last time. */
    gServoDeviceData.m_DeviceCurrentButtonState = (gServoDeviceData.m_DeviceCurrentButtons & HD_DEVICE_BUTTON_1) ? HD_TRUE : HD_FALSE;
    gServoDeviceData.m_DeviceLastButtonState = (gServoDeviceData.m_DeviceLastButtons & HD_DEVICE_BUTTON_1) ? HD_TRUE : HD_FALSE;
    
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
    
    /* Also check the error state of HDAPI. */
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

/********************************************************************
    Main scheduler callback for switching command mode.
*********************************************************************/
HDCallbackCode HDCALLBACK SwitchCommandModeCallback(void *pUserData)
{
    /* If the user presses the gimbal button, calculate the incremental displacement(currentPose - initialPose),
        and send the displacement to slave robot for incremental position control. */
    if (gServoDeviceData.m_DeviceCurrentButtonState && !gROSConfig.m_IncrementalCommandFlag)
    {
        gROSConfig.m_IncrementalCommandFlag = HD_TRUE;
        gROSConfig.m_InitialPositionCommand = gServoDeviceData.m_DeviceCurrentPosition;
        gROSConfig.m_InitialOrientationCommand = gServoDeviceData.m_DeviceCurrentGimbalAngles;
    }
    else if (gServoDeviceData.m_DeviceCurrentButtonState && gROSConfig.m_IncrementalCommandFlag)
    {
        hduVecSet(gROSConfig.m_PositionCommand, 
                    gServoDeviceData.m_DeviceCurrentPosition[0] - gROSConfig.m_InitialPositionCommand[0], 
                    gServoDeviceData.m_DeviceCurrentPosition[1] - gROSConfig.m_InitialPositionCommand[1], 
                    gServoDeviceData.m_DeviceCurrentPosition[2] - gROSConfig.m_InitialPositionCommand[2]);
        hduVecSet(gROSConfig.m_OrientationCommand, 
                    gServoDeviceData.m_DeviceCurrentGimbalAngles[0] - gROSConfig.m_InitialOrientationCommand[0], 
                    gServoDeviceData.m_DeviceCurrentGimbalAngles[1] - gROSConfig.m_InitialOrientationCommand[1], 
                    gServoDeviceData.m_DeviceCurrentGimbalAngles[2] - gROSConfig.m_InitialOrientationCommand[2]);
        gROSConfig.command_index += 1;
        fprintf(stdout, "Cmd rel position(x,y,z): (%g, %g, %g)\n",
            gROSConfig.m_PositionCommand[0],
            gROSConfig.m_PositionCommand[1],
            gROSConfig.m_PositionCommand[2]);
        fprintf(stdout, "Cmd rel orientation(yaw,pitch,roll): (%g, %g, %g)\n",
            gROSConfig.m_OrientationCommand[0],
            gROSConfig.m_OrientationCommand[1],
            gROSConfig.m_OrientationCommand[2]);
    }
    else if (!gServoDeviceData.m_DeviceCurrentButtonState)
    {
        gROSConfig.m_IncrementalCommandFlag = HD_FALSE;
        hduVecSet(gROSConfig.m_PositionCommand, 0.0, 0.0, 0.0);
        hduVecSet(gROSConfig.m_OrientationCommand, 0.0, 0.0, 0.0);
        gROSConfig.command_index = 0;
        fprintf(stdout, "Gimbal button release...\n");
    }
    
    /* Check if an error occurred while attempting to switch the command mode. */
    if (HD_DEVICE_ERROR(gServoDeviceData.m_DeviceError))
    {
        if (hduIsSchedulerError(&gServoDeviceData.m_DeviceError))
        {
            hduPrintError(stderr, &gServoDeviceData.m_DeviceError, "Error detected while switching the command mode.\n");
            return HD_CALLBACK_DONE;
        }
    }
    
    return HD_CALLBACK_DONE;
}

/************************************************
    Main loop for user application.
*************************************************/
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
        
        /* Schedule the publish callback function for publishing the device state. */
        hdScheduleSynchronous(PublishDeviceStateCallback, 0, HD_DEFAULT_SCHEDULER_PRIORITY);
        
        /* Schedule the switch callback function for switching the command mode. */
        hdScheduleSynchronous(SwitchCommandModeCallback, 0, HD_DEFAULT_SCHEDULER_PRIORITY);
        
        /* Check if the main scheduler callback has exited. */
        if (!hdWaitForCompletion(gQueryDeviceStateSchedulerHandle, HD_WAIT_CHECK_STATUS))
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
    printf("\nTouch_Common(Package): Touch_Query_State(Node)\n\n");
    ConfigureROS(argc, argv);
    
    /* Initialize the device, must be done before attempting to call any hd functions. */
    HDErrorInfo error;
    HHD hHD = hdInitDevice(HD_DEFAULT_DEVICE);
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error, "Failed to initialize the device");
        fprintf(stderr, "\nPress any key to quit.\n");
        getch();
        return -1;           
    }
    
    /* Schedule the main scheduler callback that query the device state. */
    gQueryDeviceStateSchedulerHandle = hdScheduleAsynchronous(QueryDeviceStateCallback, 0, HD_MAX_SCHEDULER_PRIORITY);
    
    /* Start the servo loop scheduler. */
    hdStartScheduler();
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error, "Failed to start the scheduler");
        fprintf(stderr, "\nPress any key to quit.\n");
        getch();
        return -1;           
    }
    
    /* Print out the help infos about this node. */
    PrintHelp();
    
    /* Run the application loop. */
    MainLoop();
    
    /* For cleanup, unschedule callbacks and stop the servo loop. */
    hdStopScheduler();
    hdUnschedule(gQueryDeviceStateSchedulerHandle);
    hdDisableDevice(hHD);
    
    return 0;
}
