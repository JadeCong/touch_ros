# ROS Packages for 3D Systems Touch

## 1. Package touch_description

> 功能：对Touch进行运动学建模，并输出其模型描述urdf文件。<br>
> 启动：roslaunch touch_description touch_display.launch

## 2. Package touch_msgs

> 功能：针对Touch生成特定的类型的消息格式，为Touch及其他设备提供特定的消息类型。

## 3. Package touch_common

> 功能：测试Touch正常的数据采集功能，并根据可Gimbal上按钮来切换模式。<br>
> 启动：roslaunch touch_common touch_query_state.launch

## 4. Package touch_teleoperation

> 功能：作为遥操作的主操作手节点，完成以下三项功能：（1）采集Touch的位姿作为控制指令发送给从操作手；（2）接收从操作手末端的反馈力并将其映射到主操作手的对应的工作空间；（3）将映射后反馈力施加到主操作手输出给操作者感知。<br>
> 启动：
>> (1)遥操作主操作手节点（单独功能）：roslaunch touch_teleoperation master_touch.launch <br>
>> (2)感知Hand的反馈力（单独功能）：roslaunch touch_teleoperation feedback_force_from_hand.launch
