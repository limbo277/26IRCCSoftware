//
// Created by LIMBO on 2026/4/21.
//
#include "graysensor.h"
#include "bsp_log.h"
#include "cmsis_os.h"
#include "grayscale.h"
#include "message_center.h"
#include "usart.h"
#include "vofa/vofa_usart.h"
// 消息发布订阅
static Publisher_t *Graysensor_Pub;  // 发布灰度传感器的数据
static Subscriber_t *Graysensor_Sub; // 订阅灰度传感器的控制命令

static Graysensor_Ctrl_Cmd_s Graysensor_Cmd_Recv;         // 接收到的控制命令
static Graysensor_Upload_Data_s Graysensor_Feedback_Data; // 反馈数据

static GrayscaleData_t *grayscale_data = NULL; // 灰度传感器数据指针
static GrayscaleData_t grayscale_data_KF = {
    0}; // 灰度传感器数据的卡尔曼滤波后值

static KalmanFilter_t Gray_KF[8]; // 灰度传感器数据的卡尔曼滤波器实例

static GrayCalib_t GrayCalib[6]; // 灰度传感器的校准数据,最多支持6套校准数据

void Graysensor_Kalman_Init() {
  for (int i = 0; i < 8; i++) {
    Kalman_Filter_Init(&Gray_KF[i], 1, 0, 1); // 1维状态, 0维控制, 1维量测
    Gray_KF[i].F_data[0] = 1.0f;              // 状态转移矩阵 F = [1]
    Gray_KF[i].P_data[0] = 10.0f;             // 初始协方差 P = [10]
    Gray_KF[i].Q_data[0] = 0.001f;            // 过程噪声 Q = [0.001]
    Gray_KF[i].R_data[0] = 50.0f;             // 量测噪声 R = [50]
    Gray_KF[i].H_data[0] = 1.0f;          // 观测矩阵 H = [1]
    Gray_KF[i].StateMinVariance[0] = 0.01f;   // 最小方差抑制过度收敛
    Gray_KF[i].UseAutoAdjustment = 0;         // 关闭自动调整（简单场景）
  }
}
/*
 * @brief 灰度传感器数据卡尔曼滤波
 * 对每个通道的原始数据进行卡尔曼滤波，更新滤波后的数据
 * */
void Graysensor_Kalman_Filter() {
  for (int i = 0; i < 8; i++) {
    Gray_KF[i].MeasuredVector[0] = grayscale_data->sensor_values[i];
    grayscale_data_KF.sensor_values[i] =
        (uint16_t)Kalman_Filter_Update(&Gray_KF[i]);
  }
}
/*
 * @brief 灰度传感器校准数据初始化
 * 预设6套校准数据,分别对应X0与X2-X6一共六个通道的数据
 */
void GrayCalibInit() {
  // 预设6套校准数据,分别对应X0与X2-X6一共六个通道的数据
  // X0通道数据
  GrayCalib[0].black_base = 3635; // 灰度0
  GrayCalib[0].white_base = 2490;
  // X2通道数据
  GrayCalib[1].black_base = 3746; // 灰度2
  GrayCalib[1].white_base = 1963;
  // X3通道数据
  GrayCalib[2].black_base = 3765; // 灰度2
  GrayCalib[2].white_base = 2015;
  // X4通道数据
  GrayCalib[3].black_base = 3748; // 灰度3
  GrayCalib[3].white_base = 1946;

  // X5通道数据
  GrayCalib[4].black_base = 3769; // 灰度4
  GrayCalib[4].white_base = 2104;

  // X6通道数据
  GrayCalib[5].black_base = 3795; // 灰度5
  GrayCalib[5].white_base = 2333;
}

/*
 * @brief 灰度传感器数据归一化
 * 根据预设的黑白基准值将原始传感器值归一化
 * */
void Gray_Normalize() {
  for (int i = 0; i < 6; i++) {

    Graysensor_Feedback_Data.sensor_Normalized[i] =
        (float)(Graysensor_Feedback_Data.sensor_values[i]- GrayCalib[i].white_base)/(float)
        (GrayCalib[i].black_base-GrayCalib[i].white_base);
  }
}

/*
 * @brief 灰度传感器应用初始化
 * 请在开启RTOS之前调用
 */
void GraysensorInit() {

  // 初始化灰度传感器 (使用USART8)
  extern UART_HandleTypeDef huart10;
  grayscale_data = GrayscaleInit(&huart10);

  // 初始化卡尔曼滤波器
  Graysensor_Kalman_Init();
  // 初始化校准数据
  GrayCalibInit();
  // 注册消息发布订阅
  Graysensor_Sub = SubRegister("Graysensor_Cmd", sizeof(Graysensor_Ctrl_Cmd_s));
  Graysensor_Pub =
      PubRegister("Graysensor_Feed", sizeof(Graysensor_Upload_Data_s));

  // 发送默认控制命令 (模拟模式)
  GrayscaleSendControl(0, 1, 0);
}

/*
 * @brief 灰度传感器应用任务
 * 放入实时系统以一定频率运行
 */
void GraysensorTask() {
  // 获取控制命令
  SubGetMessage(Graysensor_Sub, &Graysensor_Cmd_Recv);

  if (tx_flag == 0) {
    GrayscaleSendControl(0, 1, 0);
  }

  // 根据控制命令设置传感器模式
  // GrayscaleSendControl(Graysensor_Cmd_Recv.calib_mode,
  //                     Graysensor_Cmd_Recv.analog_mode,
  //                     Graysensor_Cmd_Recv.digital_mode);

  // 检查是否有新数据包
  if (grayscale_data && grayscale_data->new_package_flag) {
    Graysensor_Kalman_Filter();
    // 填充反馈数据
    for (int i = 0; i < 6; i++) {
      Graysensor_Feedback_Data.sensor_values[i] =
          grayscale_data_KF.sensor_values[i];
    }
    Gray_Normalize();

    Graysensor_Feedback_Data.data_valid = grayscale_data->valid;
    Graysensor_Feedback_Data.sensor_online = GrayscaleIsOnline();

    // 清除标志位
    GrayscaleClearNewPackageFlag();

    // // 使用阻塞发送灰度传感器数据
    // char tx_buf[128];
    // int len = sprintf(tx_buf, "%d,%d,%d,%d,%d,%d,%d,%d\r\n",
    //                   Graysensor_Feedback_Data.sensor_values[0],
    //                   Graysensor_Feedback_Data.sensor_values[1],
    //                   Graysensor_Feedback_Data.sensor_values[2],
    //                   Graysensor_Feedback_Data.sensor_values[3],
    //                   Graysensor_Feedback_Data.sensor_values[4],
    //                   Graysensor_Feedback_Data.sensor_values[5],
    //                   Graysensor_Feedback_Data.sensor_values[6],
    //                   Graysensor_Feedback_Data.sensor_values[7]);
    // HAL_UART_Transmit(&huart8, (uint8_t *)tx_buf, len, 100);
  }

  osDelay(10);
  // 发布反馈数据
  PubPushMessage(Graysensor_Pub, (void *)&Graysensor_Feedback_Data);
}
