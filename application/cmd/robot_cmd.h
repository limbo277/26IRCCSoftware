#ifndef ROBOT_CMD_H
#define ROBOT_CMD_H
#include "stdint.h"
#include "stdbool.h"

/* AGV 速度常量（需根据实际调试修改） */
#define AGV_APPROACH_SPEED       0.3f   // 接近平台速度
#define AGV_START_SPEED          0.8f   // 冲台速度
#define UPLOAD_PLATFORM_SPEED    0.2f   // 低速爬台速度
#define UPLOAD_PLATFORM_DISTANCE 5.0f   // 到台沿距离 单位cm
#define CLIMB_TIMEOUT_MS         3000   // 爬台超时 ms
#define GRAY_CONFIRM_MS          200    // 灰度确认上台持续时间 ms
#define YAW_DEVIATION_THRESHOLD  30.0f  // Yaw 偏航阈值单位为度
#define PITCH_LEVEL_THRESHOLD    1.0f   // Pitch 水平判定阈值单位为度
#define MAX_RETRY_COUNT          3      // 最大重试次数
#define UPLOAD_PLATFORM_LOWER_THRESHOLD 0.5f   // 爬台过程中灰度值的低阈值，单位为0-1

#define AGV_FALL_DOWN_PLATFORM_ACCEL_Z -24.0f

/*AGV模式对边距离常量*/
#define AGV_LASER_DISTANCE_FL 200
#define AGV_LASER_DISTANCE_FR 200
#define AGV_LASER_DISTANCE_BL 200
#define AGV_LASER_DISTANCE_BR 200


typedef enum {
  AGV_Mode_FLLOW = 0, // 跟随模式，视觉系统跟随目标物体，底盘自动调整航向和速度以保持目标在视野中心
  AGV_Mode_Start = 1, // 启动模式，底盘自动调整航向和速度以返回平台
  AGV_Mode_SCANPLATFORM, // 扫地平台模式，视觉系统扫描地面寻找特定标志物，底盘自动调整航向和速度以覆盖扫描区域
  AGV_Mode_ATTACK, // 攻击模式，视觉系统攻击目标物体，底盘自动调整航向和速度以攻击目标
  AGV_Mode_OBSTACLE, // 障碍物模式，视觉系统检测到障碍物，底盘自动调整航向和速度以避免障碍物
  AGV_Mode_ANTIDROP , // 防止掉落模式，底盘自动调整航向和速度以避免掉落
  AGV_Mode_RETURNPLATFORM, // 返回平台模式，掉台后返回平台，底盘自动调整航向和速度以返回平台
  AGV_Mode_ZERO_FORCE, // 零力模式，底盘停止运动，IMU复位
}AGV_Mode_e;

typedef struct {
  AGV_Mode_e Mode;
  uint8_t Priority;//数字越大优先级越高，优先级相同则按照默认顺序执行，一共10级优先级
  bool StartFlag; // 启动模式标志位，用于启动模式，首次启动后为False，上台后即置为True
}AGV_Mode_t;

/**
 * @brief 机器人核心控制任务初始化,会被RobotInit()调用
 * 
 */
void RobotCMDInit();

/**
 * @brief 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率)
 * 
 */
void RobotCMDTask();

#endif // !ROBOT_CMD_H