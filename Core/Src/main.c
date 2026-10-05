/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MOTOR_A_ID                 1U
#define MOTOR_B_ID                 2U

#define MOTOR_MODE_UNKNOWN         0U
#define MOTOR_MODE_VOLTAGE         1U
#define MOTOR_MODE_CURRENT         2U

// 现场确认后，再选择 VOLTAGE 或 CURRENT
#define MOTOR_CONTROL_MODE         MOTOR_MODE_UNKNOWN

#if (MOTOR_A_ID < 1U) || (MOTOR_A_ID > 7U)
#error "MOTOR_A_ID must be 1..7"
#endif

#if (MOTOR_B_ID < 1U) || (MOTOR_B_ID > 7U)
#error "MOTOR_B_ID must be 1..7"
#endif

#if MOTOR_A_ID == MOTOR_B_ID
#error "Motor A and B must have different IDs"
#endif
#define CONTROL_MODE_DISABLED      0U
#define CONTROL_MODE_LINK          1U
#define CONTROL_MODE_RESET         2U

// 0：未确认；1：右拨杆对应 rc_s1；2：对应 rc_s2
#define RC_RIGHT_SWITCH_SOURCE     0U

#if RC_RIGHT_SWITCH_SOURCE > 2U
#error "RC_RIGHT_SWITCH_SOURCE must be 0, 1 or 2"
#endif
// 待现场确认电机模式后，再逐步调试这些参数
#define MOTOR_ANGLE_KP       0.0f
#define MOTOR_SPEED_KP       0.0f
#define MOTOR_MAX_RPM        0.0f
#define MOTOR_MAX_COMMAND    0.0f
// 现场能够保证每次启动时三个箭头对齐后，才改为 1
#define MOTOR_STARTUP_ARROWS_ALIGNED    0U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// 发送期间保持内容不变
static char uart_tx_buf[768];
// DMA 将接收到的遥控器原始数据存放在这里
static uint8_t rc_rx_buf[64];

// 用于观察接收情况
static volatile uint16_t rc_rx_size = 0;
static volatile uint32_t rc_rx_count = 0;
// 四个摇杆通道，回中时接近 0
static volatile int16_t rc_ch[4] = {0};

// 两个拨杆的位置
static volatile uint8_t rc_s1 = 0;
static volatile uint8_t rc_s2 = 0;

// 用于后续判断是否持续收到有效数据
static volatile uint32_t rc_last_tick = 0;
static volatile uint8_t rc_received = 0;
// 接收出错时，由主循环负责恢复
static volatile uint8_t rc_need_restart = 0;
static uint8_t imu_ready = 0;
// 加速度，单位 m/s²
static float imu_ax = 0;
static float imu_ay = 0;
static float imu_az = 0;

// 角速度，单位 °/s
static float imu_gx = 0;
static float imu_gy = 0;
static float imu_gz = 0;
typedef struct
{
    uint16_t encoder;
    int32_t total_encoder;  // 相对首次反馈位置的累计计数
    uint8_t angle_valid;    // 连续角度是否有效       // 单圈编码器值：0～8191
    int16_t speed_rpm;      // 转速：转/分钟，有正负
    uint8_t temperature;    // 温度：℃
    uint8_t received;       // 是否曾收到有效反馈
    uint32_t last_rx_tick;  // 最近一次接收时间
    uint32_t rx_count;      // 累计接收帧数
} MotorFeedback;

// 下标直接对应电机 ID，motor_fb[0] 不使用
static volatile MotorFeedback motor_fb[8] = {0};
// 陀螺仪零偏，单位 °/s
static float gyro_bias_x = 0.0f;
static float gyro_bias_y = 0.0f;
static float gyro_bias_z = 0.0f;

// 连续采集 200 个静止样本
static uint16_t imu_cal_count = 0;
static uint8_t imu_cal_ready = 0;
static float imu_yaw_deg = 0.0f;       // 连续 yaw，单位度
static uint8_t imu_attitude_ready = 0;
static uint8_t imu_attitude_fault = 0;
// 遥控器请求的工作模式
static uint8_t control_requested_mode = CONTROL_MODE_DISABLED;

// B 相对于 A/C 板的联动比例
static float motor_b_ratio = 0.5f;

// 是否具备进入电机控制的基本条件
static uint8_t motor_control_allowed = 0U;
// 0：正常联动；1：手动转 A；2：手动转 B
static uint8_t manual_leader = 0U;
static uint8_t manual_armed = 0U;
static uint8_t manual_candidate = 0U;

static uint16_t manual_settle_count = 0U;
static uint16_t manual_candidate_count = 0U;
static uint16_t manual_release_count = 0U;

static uint8_t arrow_reference_ready = 0U;

// 箭头对齐时，电机连续角度与 IMU yaw 的差值
static float arrow_offset_a = 0.0f;
static float arrow_offset_b = 0.0f;
// 记录上一批电机报文使用的发送邮箱
static uint32_t motor_tx_mailboxes = 0U;

// 发送故障锁定，复位 C 板后清除
static uint8_t motor_tx_fault = 0U;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void LED_Show(uint8_t index)
{
    // 先关闭全部 LED
    HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_SET);

    // 再点亮指定的 LED
    switch (index)
    {
        case 0:
            HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin,
                              GPIO_PIN_RESET);
            break;

        case 1:
            HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin,
                              GPIO_PIN_RESET);
            break;

        case 2:
            HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin,
                              GPIO_PIN_RESET);
            break;

        default:
            break;
    }
}
// is_acc：1 表示加速度计，0 表示陀螺仪
static HAL_StatusTypeDef BMI088_WriteReg(
    uint8_t is_acc, uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = {reg & 0x7FU, value};

    GPIO_TypeDef *port = is_acc ? ACC_CS_GPIO_Port
                               : GYRO_CS_GPIO_Port;
    uint16_t pin = is_acc ? ACC_CS_Pin : GYRO_CS_Pin;

    HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);

    HAL_StatusTypeDef status =
        HAL_SPI_Transmit(&hspi1, tx, sizeof(tx), 10);

    HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);

    // 留出寄存器写入间隔，仅在初始化时使用
    HAL_Delay(2);

    return status;
}
static HAL_StatusTypeDef BMI088_Init(void)
{
    // 两个传感器先都不选中
    HAL_GPIO_WritePin(ACC_CS_GPIO_Port, ACC_CS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_SET);

    HAL_Delay(50);

    // 首次空读：使加速度计进入 SPI 模式，不检查芯片 ID
    uint8_t tx[3] = {0x80, 0x00, 0x00};
    uint8_t rx[3];

    HAL_GPIO_WritePin(ACC_CS_GPIO_Port, ACC_CS_Pin, GPIO_PIN_RESET);

    HAL_StatusTypeDef status =
        HAL_SPI_TransmitReceive(&hspi1, tx, rx, sizeof(tx), 10);

    HAL_GPIO_WritePin(ACC_CS_GPIO_Port, ACC_CS_Pin, GPIO_PIN_SET);

    if (status != HAL_OK)
    {
        return status;
    }

    HAL_Delay(2);

    // 加速度计：退出挂起状态
    if (BMI088_WriteReg(1, 0x7C, 0x00) != HAL_OK)
        return HAL_ERROR;

    HAL_Delay(5);

    // 开启加速度测量
    if (BMI088_WriteReg(1, 0x7D, 0x04) != HAL_OK)
        return HAL_ERROR;

    HAL_Delay(5);

    // 加速度计：100 Hz 输出数据率
    if (BMI088_WriteReg(1, 0x40, 0xA8) != HAL_OK)
        return HAL_ERROR;

    // 加速度计量程：±6 g
    if (BMI088_WriteReg(1, 0x41, 0x01) != HAL_OK)
        return HAL_ERROR;

    // 陀螺仪：正常工作模式
    if (BMI088_WriteReg(0, 0x11, 0x00) != HAL_OK)
        return HAL_ERROR;

    HAL_Delay(30);

    // 陀螺仪量程：±2000 °/s
    if (BMI088_WriteReg(0, 0x0F, 0x00) != HAL_OK)
        return HAL_ERROR;

    // 陀螺仪：100 Hz 输出数据率，32 Hz 带宽
    if (BMI088_WriteReg(0, 0x10, 0x07) != HAL_OK)
        return HAL_ERROR;

    HAL_Delay(30);

    return HAL_OK;
}
static HAL_StatusTypeDef BMI088_ReadRaw(
    uint8_t is_acc, int16_t raw[3])
{
    // 加速度数据从 0x12 开始，陀螺仪数据从 0x02 开始
    uint8_t tx[8] = {0};
    uint8_t rx[8] = {0};

    tx[0] = (is_acc ? 0x12U : 0x02U) | 0x80U;

    // 加速度计比陀螺仪多一个无效字节，需要跳过
    uint16_t length = is_acc ? 8U : 7U;
    uint8_t offset = is_acc ? 2U : 1U;

    GPIO_TypeDef *port = is_acc ? ACC_CS_GPIO_Port
                               : GYRO_CS_GPIO_Port;
    uint16_t pin = is_acc ? ACC_CS_Pin : GYRO_CS_Pin;

    HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);

    HAL_StatusTypeDef status =
        HAL_SPI_TransmitReceive(&hspi1, tx, rx, length, 10);

    HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);

    if (status != HAL_OK)
    {
        return status;
    }

    for (uint8_t i = 0; i < 3U; i++)
    {
        uint8_t pos = offset + 2U * i;

        // 每个轴由低字节、高字节组成一个有符号 16 位数
        uint16_t value = (uint16_t)rx[pos] |
                         ((uint16_t)rx[pos + 1U] << 8);

        raw[i] = (int16_t)value;
    }

    return HAL_OK;
}
static HAL_StatusTypeDef BMI088_ReadData(void)
{
    int16_t acc_raw[3];
    int16_t gyro_raw[3];

    if (BMI088_ReadRaw(1, acc_raw) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (BMI088_ReadRaw(0, gyro_raw) != HAL_OK)
    {
        return HAL_ERROR;
    }

    // 对应初始化设置的加速度量程 ±6 g
    const float acc_scale = 6.0f * 9.80665f / 32768.0f;

    // 对应初始化设置的陀螺仪量程 ±2000 °/s
    const float gyro_scale = 2000.0f / 32768.0f;

    // 两组读取都成功后才更新结果
    imu_ax = acc_raw[0] * acc_scale;
    imu_ay = acc_raw[1] * acc_scale;
    imu_az = acc_raw[2] * acc_scale;

    imu_gx = gyro_raw[0] * gyro_scale;
    imu_gy = gyro_raw[1] * gyro_scale;
    imu_gz = gyro_raw[2] * gyro_scale;

    return HAL_OK;
}
static void IMU_UpdateGyroCalibration(uint8_t sample_ok)
{
    static float sum_x = 0.0f;
    static float sum_y = 0.0f;
    static float sum_z = 0.0f;

    // 完成后不再自动重新校准
    if (imu_cal_ready)
    {
        return;
    }

    float acc_sq = imu_ax * imu_ax +
                   imu_ay * imu_ay +
                   imu_az * imu_az;

    // 初步静止检查：
    // 加速度合量接近重力，三个轴角速度都较小
    uint8_t stationary =
        sample_ok &&
        acc_sq > (8.8f * 8.8f) &&
        acc_sq < (10.8f * 10.8f) &&
        imu_gx > -3.0f && imu_gx < 3.0f &&
        imu_gy > -3.0f && imu_gy < 3.0f &&
        imu_gz > -3.0f && imu_gz < 3.0f;

    // 读取失败或检测到明显运动，重新累计
    if (!stationary)
    {
        sum_x = 0.0f;
        sum_y = 0.0f;
        sum_z = 0.0f;
        imu_cal_count = 0;
        return;
    }

    sum_x += imu_gx;
    sum_y += imu_gy;
    sum_z += imu_gz;
    imu_cal_count++;

    if (imu_cal_count >= 200U)
    {
        gyro_bias_x = sum_x / (float)imu_cal_count;
        gyro_bias_y = sum_y / (float)imu_cal_count;
        gyro_bias_z = sum_z / (float)imu_cal_count;

        imu_cal_ready = 1U;
    }
}
static void IMU_UpdateAttitude(uint8_t sample_ok, float dt)
{
    // 四元数：描述 C 板相对于参考坐标系的姿态
    static float q0 = 1.0f;
    static float q1 = 0.0f;
    static float q2 = 0.0f;
    static float q3 = 0.0f;
    static float last_yaw = 0.0f;

    const float deg_to_rad = 0.01745329252f;
    const float rad_to_deg = 57.29577951f;

    if (!imu_cal_ready || imu_attitude_fault)
    {
        return;
    }

    // 丢失较长时间的数据后，不继续输出看似有效的角度
    if (!sample_ok || dt <= 0.0f || dt > 0.05f)
    {
        imu_attitude_ready = 0U;
        imu_attitude_fault = 1U;
        return;
    }

    float ax = imu_ax;
    float ay = imu_ay;
    float az = imu_az;
    float acc_sq = ax * ax + ay * ay + az * az;

    if (!imu_attitude_ready)
    {
        // 用静止时的重力方向初始化倾斜姿态
        // 接近俯仰 ±90° 时，yaw 方向不适合作为联动输入
        if (acc_sq < 1.0f ||
            (ay * ay + az * az) / acc_sq < 0.01f)
        {
            imu_attitude_fault = 1U;
            return;
        }

        float roll = atan2f(ay, az);
        float pitch = atan2f(-ax, sqrtf(ay * ay + az * az));

        float cr = cosf(roll * 0.5f);
        float sr = sinf(roll * 0.5f);
        float cp = cosf(pitch * 0.5f);
        float sp = sinf(pitch * 0.5f);

        // 初始 yaw 定义为 0
        q0 = cr * cp;
        q1 = sr * cp;
        q2 = cr * sp;
        q3 = -sr * sp;

        last_yaw = atan2f(
            2.0f * (q0 * q3 + q1 * q2),
            1.0f - 2.0f * (q2 * q2 + q3 * q3)) * rad_to_deg;

        imu_yaw_deg = 0.0f;
        imu_attitude_ready = 1U;
        return;
    }

    // 减去零偏，并将 °/s 转换为 rad/s
    float gx = (imu_gx - gyro_bias_x) * deg_to_rad;
    float gy = (imu_gy - gyro_bias_y) * deg_to_rad;
    float gz = (imu_gz - gyro_bias_z) * deg_to_rad;

    // 加速度合量接近重力时，才进行倾斜修正
    if (acc_sq > 8.8f * 8.8f &&
        acc_sq < 10.8f * 10.8f)
    {
        float inv_acc = 1.0f / sqrtf(acc_sq);
        ax *= inv_acc;
        ay *= inv_acc;
        az *= inv_acc;

        // 当前姿态预测的重力方向
        float vx = 2.0f * (q1 * q3 - q0 * q2);
        float vy = 2.0f * (q0 * q1 + q2 * q3);
        float vz = q0 * q0 - q1 * q1 -
                   q2 * q2 + q3 * q3;

        // 测量方向与预测方向的叉积误差
        float ex = ay * vz - az * vy;
        float ey = az * vx - ax * vz;
        float ez = ax * vy - ay * vx;

        const float kp = 2.0f;
        gx += kp * ex;
        gy += kp * ey;
        gz += kp * ez;
    }

    // 必须先保存旧值，再同时更新四个分量
    float a = q0;
    float b = q1;
    float c = q2;
    float d = q3;
    float half_dt = 0.5f * dt;

    q0 += (-b * gx - c * gy - d * gz) * half_dt;
    q1 += ( a * gx + c * gz - d * gy) * half_dt;
    q2 += ( a * gy - b * gz + d * gx) * half_dt;
    q3 += ( a * gz + b * gy - c * gx) * half_dt;

    float norm_sq = q0 * q0 + q1 * q1 +
                    q2 * q2 + q3 * q3;

    if (!isfinite(norm_sq) || norm_sq < 0.000001f)
    {
        imu_attitude_ready = 0U;
        imu_attitude_fault = 1U;
        return;
    }

    float inv_norm = 1.0f / sqrtf(norm_sq);
    q0 *= inv_norm;
    q1 *= inv_norm;
    q2 *= inv_norm;
    q3 *= inv_norm;

    float heading_x =
        1.0f - 2.0f * (q2 * q2 + q3 * q3);
    float heading_y =
        2.0f * (q0 * q3 + q1 * q2);

    if (heading_x * heading_x +
        heading_y * heading_y < 0.01f)
    {
        imu_attitude_ready = 0U;
        imu_attitude_fault = 1U;
        return;
    }

    float yaw = atan2f(heading_y, heading_x) * rad_to_deg;
    float delta = yaw - last_yaw;

    // 处理 +180° 与 -180° 之间的跳变，累计连续角度
    if (delta > 180.0f)
    {
        delta -= 360.0f;
    }
    else if (delta < -180.0f)
    {
        delta += 360.0f;
    }

    imu_yaw_deg += delta;
    last_yaw = yaw;
}
// 把数值限制在 -limit～+limit
static float Motor_Clamp(float value, float limit)
{
    if (value > limit)
    {
        return limit;
    }

    if (value < -limit)
    {
        return -limit;
    }

    return value;
}

// 位置外环 + 速度内环，当前先使用比例控制
static int16_t Motor_PositionControl(
    float target_deg,
    float actual_deg,
    float actual_rpm,
    float angle_kp,
    float speed_kp,
    float max_rpm,
    float max_command)
{
    // 参数未设置或数据异常时，不产生驱动给定
    if (!isfinite(target_deg) ||
        !isfinite(actual_deg) ||
        !isfinite(actual_rpm) ||
        !isfinite(angle_kp) ||
        !isfinite(speed_kp) ||
        !isfinite(max_rpm) ||
        !isfinite(max_command) ||
        angle_kp <= 0.0f ||
        speed_kp <= 0.0f ||
        max_rpm <= 0.0f ||
        max_command <= 0.0f ||
        max_command > 25000.0f)
    {
        return 0;
    }

    // 外环：角度误差转换成目标转速
    float angle_error = target_deg - actual_deg;

    float target_rpm =
        Motor_Clamp(angle_kp * angle_error, max_rpm);

    // 内环：转速误差转换成控制给定值
    float speed_error = target_rpm - actual_rpm;

    float command =
        Motor_Clamp(speed_kp * speed_error, max_command);

    return (int16_t)command;
}
static HAL_StatusTypeDef Motor_SendCommandsRaw(
    int16_t command_a, int16_t command_b)
{
    uint32_t group_id[2];
    int32_t limit;

    // 未确认模式时，不发送控制报文
    if (MOTOR_CONTROL_MODE == MOTOR_MODE_VOLTAGE)
    {
        group_id[0] = 0x1FFU;  // ID 1～4
        group_id[1] = 0x2FFU;  // ID 5～7
        limit = 25000;
    }
    else if (MOTOR_CONTROL_MODE == MOTOR_MODE_CURRENT)
    {
        group_id[0] = 0x1FEU;
        group_id[1] = 0x2FEU;
        limit = 16384;
    }
    else
    {
        return HAL_ERROR;
    }

    uint8_t data[2][8] = {{0}};
    uint8_t group_used[2] = {0};
    const uint8_t ids[2] = {MOTOR_A_ID, MOTOR_B_ID};
    int32_t commands[2] = {command_a, command_b};

    for (uint8_t i = 0; i < 2U; i++)
    {
        // 限制在协议允许的范围内
        if (commands[i] > limit)
        {
            commands[i] = limit;
        }
        else if (commands[i] < -limit)
        {
            commands[i] = -limit;
        }

        uint8_t group = (uint8_t)((ids[i] - 1U) / 4U);
        uint8_t slot = (uint8_t)((ids[i] - 1U) % 4U);
        uint8_t offset = (uint8_t)(slot * 2U);

        uint16_t value = (uint16_t)(int16_t)commands[i];

        data[group][offset] = (uint8_t)(value >> 8);
        data[group][offset + 1U] = (uint8_t)value;
        group_used[group] = 1U;
    }

    uint32_t needed =
        (uint32_t)group_used[0] + (uint32_t)group_used[1];

    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) < needed)
    {
        return HAL_BUSY;
    }

    CAN_TxHeaderTypeDef header = {0};
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = 8U;
    header.TransmitGlobalTime = DISABLE;

    for (uint8_t group = 0; group < 2U; group++)
    {
        if (!group_used[group])
        {
            continue;
        }

        header.StdId = group_id[group];
        uint32_t mailbox;

        HAL_StatusTypeDef status = HAL_CAN_AddTxMessage(
            &hcan1, &header, data[group], &mailbox);

        if (status != HAL_OK)
        {
            return status;
        }
        motor_tx_mailboxes |= mailbox;
    }

    return HAL_OK;
}
static HAL_StatusTypeDef Motor_SendCommands(
    int16_t command_a, int16_t command_b)
{
    // 模式未确认，不发送，也不因此记录发送故障
    if (MOTOR_CONTROL_MODE != MOTOR_MODE_VOLTAGE &&
        MOTOR_CONTROL_MODE != MOTOR_MODE_CURRENT)
    {
        return HAL_ERROR;
    }

    // 检查上一批发送
    if (motor_tx_mailboxes != 0U)
    {
        if (HAL_CAN_IsTxMessagePending(&hcan1, motor_tx_mailboxes))
        {
            // 到下一个控制周期仍未完成：
            // 锁定故障并请求撤销旧报文
            motor_tx_fault = 1U;
            (void)HAL_CAN_AbortTxRequest(&hcan1, motor_tx_mailboxes);

            // 撤销不是瞬间完成，下一周期继续检查
            return HAL_BUSY;
        }

        const uint32_t ok_flags[3] = {
            CAN_FLAG_TXOK0, CAN_FLAG_TXOK1, CAN_FLAG_TXOK2
        };

        const uint32_t complete_flags[3] = {
            CAN_FLAG_RQCP0, CAN_FLAG_RQCP1, CAN_FLAG_RQCP2
        };

        for (uint8_t i = 0; i < 3U; i++)
        {
            if ((motor_tx_mailboxes & (1UL << i)) != 0U)
            {
                // 不再等待发送，不代表一定发送成功
                if (!__HAL_CAN_GET_FLAG(&hcan1, ok_flags[i]))
                {
                    motor_tx_fault = 1U;
                }

                __HAL_CAN_CLEAR_FLAG(&hcan1, complete_flags[i]);
            }
        }

        motor_tx_mailboxes = 0U;
    }

    // 一旦发生故障，之后只尝试发送零给定
    if (motor_tx_fault)
    {
        command_a = 0;
        command_b = 0;
    }

    HAL_StatusTypeDef status =
        Motor_SendCommandsRaw(command_a, command_b);

    if (status != HAL_OK)
    {
        motor_tx_fault = 1U;

        // 若只成功加入了部分报文，也撤销这些发送
        if (motor_tx_mailboxes != 0U)
        {
            (void)HAL_CAN_AbortTxRequest(
                &hcan1, motor_tx_mailboxes);
        }
    }

    return status;
}
static void Control_UpdateRequest(void)
{
    uint8_t s1, s2, received;
    uint32_t rc_tick;
    MotorFeedback motor_a;
    MotorFeedback motor_b;

    // 一次性复制中断更新的数据
    uint32_t saved_irq = __get_PRIMASK();
    __disable_irq();

    s1 = rc_s1;
    s2 = rc_s2;
    received = rc_received;
    rc_tick = rc_last_tick;
    motor_a = motor_fb[MOTOR_A_ID];
    motor_b = motor_fb[MOTOR_B_ID];

    __set_PRIMASK(saved_irq);

    uint32_t now = HAL_GetTick();

    // 默认失能，只有通过检查后才允许控制
    control_requested_mode = CONTROL_MODE_DISABLED;
    motor_control_allowed = 0U;

    if (!received || (uint32_t)(now - rc_tick) >= 100U)
    {
        return;
    }

    uint8_t right_switch;
    uint8_t left_switch;

    if (RC_RIGHT_SWITCH_SOURCE == 1U)
    {
        right_switch = s1;
        left_switch = s2;
    }
    else if (RC_RIGHT_SWITCH_SOURCE == 2U)
    {
        right_switch = s2;
        left_switch = s1;
    }
    else
    {
        // 尚未确认左右拨杆对应关系
        return;
    }

    switch (left_switch)
    {
        case 2U: motor_b_ratio = 0.5f;  break; // 下
        case 3U: motor_b_ratio = -1.0f; break; // 中
        case 1U: motor_b_ratio = 3.0f;  break; // 上
        default: return;
    }

    switch (right_switch)
    {
        case 2U:
            return; // 下档：保持失能

        case 3U:
            control_requested_mode = CONTROL_MODE_LINK;
            break;

        case 1U:
            control_requested_mode = CONTROL_MODE_RESET;
            break;

        default:
            return;
    }

    // 控制模式必须已经确认
    if (MOTOR_CONTROL_MODE != MOTOR_MODE_VOLTAGE &&
        MOTOR_CONTROL_MODE != MOTOR_MODE_CURRENT)
    {
        return;
    }

    // 姿态必须有效
    if (!imu_attitude_ready || imu_attitude_fault)
    {
        return;
    }

    // 两台电机的反馈和连续角度都必须有效
    if (!motor_a.received || !motor_b.received ||
        !motor_a.angle_valid || !motor_b.angle_valid ||
        (uint32_t)(now - motor_a.last_rx_tick) >= 50U ||
        (uint32_t)(now - motor_b.last_rx_tick) >= 50U)
    {
        return;
    }
    if (motor_tx_fault)
{
    return;
}
    motor_control_allowed = 1U;
}
// leader：1 表示手动转 A，2 表示手动转 B
// 只更新联动参考，不修改 IMU 测得的 yaw
static void Link_UpdateReference(
    uint8_t leader,
    float yaw,
    float actual_a,
    float actual_b,
    float reference_a,
    float reference_b,
    float ratio,
    float *reference_yaw)
{
    if (reference_yaw == NULL ||
        !isfinite(ratio) ||
        fabsf(ratio) < 0.001f)
    {
        return;
    }

    float yaw_change = yaw - *reference_yaw;
    float phase_change;

    if (leader == 1U)
    {
        float target_a = reference_a + yaw_change;
        phase_change = actual_a - target_a;
    }
    else if (leader == 2U)
    {
        float target_b = reference_b + ratio * yaw_change;
        phase_change = (actual_b - target_b) / ratio;
    }
    else
    {
        return;
    }

    if (isfinite(phase_change))
    {
        *reference_yaw -= phase_change;
    }
}
static void Link_ManualReset(void)
{
    manual_leader = 0U;
    manual_armed = 0U;
    manual_candidate = 0U;
    manual_settle_count = 0U;
    manual_candidate_count = 0U;
    manual_release_count = 0U;
}

static uint8_t Link_ManualStep(
    float actual_a,
    float actual_b,
    float rpm_a,
    float rpm_b,
    float reference_a,
    float reference_b,
    float ratio,
    float *reference_yaw)
{
    // 初始调试阈值，后续需要实物验证和调整
    const float settled_deg = 1.0f;
    const float takeover_deg = 3.0f;
    const float stopped_rpm = 1.0f;

    uint8_t board_still =
        fabsf(imu_gx - gyro_bias_x) < 1.0f &&
        fabsf(imu_gy - gyro_bias_y) < 1.0f &&
        fabsf(imu_gz - gyro_bias_z) < 1.0f;

    // C 板转动时，回到 C 板作为输入的联动
    if (!board_still)
    {
        Link_ManualReset();
        return 0U;
    }

    float yaw_change = imu_yaw_deg - *reference_yaw;
    float target_a = reference_a + yaw_change;
    float target_b = reference_b + ratio * yaw_change;

    float error_a = actual_a - target_a;
    float error_b = actual_b - target_b;

    // 已进入手动接管
    if (manual_leader != 0U)
    {
        Link_UpdateReference(
            manual_leader, imu_yaw_deg,
            actual_a, actual_b,
            reference_a, reference_b,
            ratio, reference_yaw);

        // 更新参考后，重新计算另一台电机的目标
        yaw_change = imu_yaw_deg - *reference_yaw;
        target_a = reference_a + yaw_change;
        target_b = reference_b + ratio * yaw_change;

        uint8_t follower_arrived =
            (manual_leader == 1U) ?
            (fabsf(actual_b - target_b) < settled_deg) :
            (fabsf(actual_a - target_a) < settled_deg);

        uint8_t both_stopped =
            fabsf(rpm_a) < stopped_rpm &&
            fabsf(rpm_b) < stopped_rpm;

        if (both_stopped && follower_arrived)
        {
            manual_release_count++;
        }
        else
        {
            manual_release_count = 0U;
        }

        // 约 300 ms 停稳后，保持更新后的目标位置
        if (manual_release_count >= 30U)
        {
            manual_leader = 0U;
            manual_candidate = 0U;
            manual_candidate_count = 0U;
            manual_release_count = 0U;
            manual_armed = 1U;
        }

        return manual_leader;
    }

    // 正常跟随完成后，才允许判断手动输入
    if (!manual_armed)
    {
        if (fabsf(error_a) < settled_deg &&
            fabsf(error_b) < settled_deg &&
            fabsf(rpm_a) < stopped_rpm &&
            fabsf(rpm_b) < stopped_rpm)
        {
            manual_settle_count++;
        }
        else
        {
            manual_settle_count = 0U;
        }

        if (manual_settle_count >= 25U)
        {
            manual_armed = 1U;
        }

        return 0U;
    }

    // 只有一台明显偏离、另一台仍在目标附近时，才建立候选
    uint8_t candidate = 0U;

    if (fabsf(error_a) > takeover_deg &&
        fabsf(error_b) < settled_deg)
    {
        candidate = 1U;
    }
    else if (fabsf(error_b) > takeover_deg &&
             fabsf(error_a) < settled_deg)
    {
        candidate = 2U;
    }

    if (candidate == 0U)
    {
        manual_candidate = 0U;
        manual_candidate_count = 0U;
        return 0U;
    }

    if (candidate != manual_candidate)
    {
        manual_candidate = candidate;
        manual_candidate_count = 1U;
    }
    else
    {
        manual_candidate_count++;
    }

    // 持续约 100 ms 后进入手动接管
    if (manual_candidate_count >= 10U)
    {
        manual_leader = candidate;
        manual_release_count = 0U;

        Link_UpdateReference(
            manual_leader, imu_yaw_deg,
            actual_a, actual_b,
            reference_a, reference_b,
            ratio, reference_yaw);
    }

    return manual_leader;
}
// 每次启动只记录一次箭头对齐参考
static void Motor_CaptureArrowReference(void)
{
    if (!MOTOR_STARTUP_ARROWS_ALIGNED ||
        arrow_reference_ready ||
        !imu_attitude_ready ||
        imu_attitude_fault)
    {
        return;
    }

    MotorFeedback a;
    MotorFeedback b;

    uint32_t saved_irq = __get_PRIMASK();
    __disable_irq();
    a = motor_fb[MOTOR_A_ID];
    b = motor_fb[MOTOR_B_ID];
    __set_PRIMASK(saved_irq);

    uint32_t now = HAL_GetTick();

    if (!a.received || !b.received ||
        !a.angle_valid || !b.angle_valid ||
        (uint32_t)(now - a.last_rx_tick) >= 50U ||
        (uint32_t)(now - b.last_rx_tick) >= 50U)
    {
        return;
    }

    float angle_a =
        (float)a.total_encoder * (360.0f / 8192.0f);
    float angle_b =
        (float)b.total_encoder * (360.0f / 8192.0f);

    arrow_offset_a = angle_a - imu_yaw_deg;
    arrow_offset_b = angle_b - imu_yaw_deg;

    arrow_reference_ready = 1U;
}

// 返回 -180°～180° 范围内的等效角度差
static float Motor_Wrap180(float angle)
{
    float result = fmodf(angle + 180.0f, 360.0f);

    if (result < 0.0f)
    {
        result += 360.0f;
    }

    return result - 180.0f;
}
static void Motor_ControlTick(void)
{
    static uint8_t link_started = 0U;
    static float reference_yaw = 0.0f;
    static float reference_a = 0.0f;
    static float reference_b = 0.0f;
    static float previous_ratio = 0.0f;
    Motor_CaptureArrowReference();

    // 允许中档联动和上档复位，其余情况发送零给定
    // 其他模式、状态异常或参数未设置时，发送零给定
    if (!motor_control_allowed ||
    (control_requested_mode != CONTROL_MODE_LINK &&
     control_requested_mode != CONTROL_MODE_RESET) ||
        MOTOR_ANGLE_KP <= 0.0f ||
        MOTOR_SPEED_KP <= 0.0f ||
        MOTOR_MAX_RPM <= 0.0f ||
        MOTOR_MAX_COMMAND <= 0.0f)
    {
        link_started = 0U;
        (void)Motor_SendCommands(0, 0);
        return;
    }

    MotorFeedback motor_a;
    MotorFeedback motor_b;

    uint32_t saved_irq = __get_PRIMASK();
    __disable_irq();
    motor_a = motor_fb[MOTOR_A_ID];
    motor_b = motor_fb[MOTOR_B_ID];
    __set_PRIMASK(saved_irq);

    uint32_t now = HAL_GetTick();

    // 对本次使用的反馈再检查一次
    if (!motor_a.received || !motor_b.received ||
        !motor_a.angle_valid || !motor_b.angle_valid ||
        (uint32_t)(now - motor_a.last_rx_tick) >= 50U ||
        (uint32_t)(now - motor_b.last_rx_tick) >= 50U)
    {
        link_started = 0U;
        (void)Motor_SendCommands(0, 0);
        return;
    }

    float actual_a =
        (float)motor_a.total_encoder * (360.0f / 8192.0f);

    float actual_b =
        (float)motor_b.total_encoder * (360.0f / 8192.0f);
        if (control_requested_mode == CONTROL_MODE_RESET)
{
    // 退出中档联动状态，回到中档时重新建立联动参考
    link_started = 0U;
    Link_ManualReset();

    if (!arrow_reference_ready)
    {
        (void)Motor_SendCommands(0, 0);
        return;
    }

    // 箭头对齐只要求方向相同，选择距离最近的等效位置
    float target_a = actual_a + Motor_Wrap180(
        imu_yaw_deg + arrow_offset_a - actual_a);

    float target_b = actual_b + Motor_Wrap180(
        imu_yaw_deg + arrow_offset_b - actual_b);

    int16_t command_a = Motor_PositionControl(
        target_a,
        actual_a,
        (float)motor_a.speed_rpm,
        MOTOR_ANGLE_KP,
        MOTOR_SPEED_KP,
        MOTOR_MAX_RPM,
        MOTOR_MAX_COMMAND);

    int16_t command_b = Motor_PositionControl(
        target_b,
        actual_b,
        (float)motor_b.speed_rpm,
        MOTOR_ANGLE_KP,
        MOTOR_SPEED_KP,
        MOTOR_MAX_RPM,
        MOTOR_MAX_COMMAND);

    (void)Motor_SendCommands(command_a, command_b);
    return;
}

    // 刚进入联动，或切换比例时，从当前位置建立参考
    // 避免直接套用上一次模式留下的目标角度
    if (!link_started || motor_b_ratio != previous_ratio)
    {
        Link_ManualReset();
        reference_yaw = imu_yaw_deg;
        reference_a = actual_a;
        reference_b = actual_b;
        previous_ratio = motor_b_ratio;
        link_started = 1U;

        (void)Motor_SendCommands(0, 0);
        return;
    }

    uint8_t manual_source = Link_ManualStep(
    actual_a,
    actual_b,
    (float)motor_a.speed_rpm,
    (float)motor_b.speed_rpm,
    reference_a,
    reference_b,
    motor_b_ratio,
    &reference_yaw);
    float yaw_change = imu_yaw_deg - reference_yaw;

    float target_a = reference_a + yaw_change;
    float target_b = reference_b + motor_b_ratio * yaw_change;

    int16_t command_a = Motor_PositionControl(
        target_a,
        actual_a,
        (float)motor_a.speed_rpm,
        MOTOR_ANGLE_KP,
        MOTOR_SPEED_KP,
        MOTOR_MAX_RPM,
        MOTOR_MAX_COMMAND);

    int16_t command_b = Motor_PositionControl(
        target_b,
        actual_b,
        (float)motor_b.speed_rpm,
        MOTOR_ANGLE_KP,
        MOTOR_SPEED_KP,
        MOTOR_MAX_RPM,
        MOTOR_MAX_COMMAND);

    
        // 被手动转动的一台不执行位置回拉，另一台继续跟随
if (manual_source == 1U)
{
    command_a = 0;
}
else if (manual_source == 2U)
{
    command_b = 0;
}
    if (Motor_SendCommands(command_a, command_b) != HAL_OK)
    {
        // 本次发送未成功进入邮箱，下次重新建立参考
        link_started = 0U;
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_SPI1_Init();
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */
if (CAN1_StartReceive() != HAL_OK)
{
    Error_Handler();
}
if (HAL_CAN_ActivateNotification(
        &hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
{
    Error_Handler();
}
imu_ready = (BMI088_Init() == HAL_OK);
uint8_t led_index = 0;                 // 0 红，1 绿，2 蓝

LED_Show(led_index);                    // 上电先亮红灯
uint32_t led_last_tick = HAL_GetTick();  // 记录开始时间
// 蜂鸣器初始化并启动 PWM
if (HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3) != HAL_OK)
{
    Error_Handler();
}

// 设置 50% 占空比，开始发声
__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 250);

uint32_t buzzer_start_tick = HAL_GetTick();
uint8_t buzzer_active = 1;
uint32_t uart_last_tick = HAL_GetTick();
uint32_t motor_last_tick = HAL_GetTick();
uint32_t imu_last_tick = HAL_GetTick();
uint8_t imu_sample_ok = 0;

// 首次启动接收；失败时交给主循环恢复
if (HAL_UARTEx_ReceiveToIdle_DMA(&huart3,
                               rc_rx_buf,
                               sizeof(rc_rx_buf)) == HAL_OK)
{
    __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);
}
else
{
    rc_need_restart = 1;
}
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t now = HAL_GetTick();
    // 出错后清理当前接收，随后重新启动
if (rc_need_restart)
{
    rc_need_restart = 0;

    if (HAL_UART_AbortReceive(&huart3) != HAL_OK)
    {
        rc_need_restart = 1;
    }
}

// 正常接收时不会进入这里；也用于重试启动失败的情况
if (!rc_need_restart)
{
    uint32_t irq_state = __get_PRIMASK();
    __disable_irq();

    if (huart3.RxState == HAL_UART_STATE_READY)
    {
        if (HAL_UARTEx_ReceiveToIdle_DMA(
                &huart3, rc_rx_buf, sizeof(rc_rx_buf)) == HAL_OK)
        {
            __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);
        }
    }

    __set_PRIMASK(irq_state);
}

// LED：每隔 300 毫秒换一个灯
if ((uint32_t)(now - led_last_tick) >= 300U)
{
    led_last_tick = now;

    led_index++;
    if (led_index >= 3U)
    {
        led_index = 0;
    }

    LED_Show(led_index);
}

// 蜂鸣器：启动响约 200 毫秒后静音
if (buzzer_active &&
    (uint32_t)(now - buzzer_start_tick) >= 200U)
{
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
    buzzer_active = 0;
}
// IMU：每隔约 10 ms 采样、校准并更新姿态
uint32_t imu_now = HAL_GetTick();

if ((uint32_t)(imu_now - imu_last_tick) >= 10U)
{
    // 根据实际采样间隔计算 dt，单位秒
    float imu_dt =
        (float)(uint32_t)(imu_now - imu_last_tick) * 0.001f;

    imu_last_tick = imu_now;
    imu_sample_ok = 0U;

    if (imu_ready)
    {
        imu_sample_ok = (BMI088_ReadData() == HAL_OK);
    }

    IMU_UpdateGyroCalibration(imu_sample_ok);
    IMU_UpdateAttitude(imu_sample_ok, imu_dt);
}
Control_UpdateRequest();
// 每隔约 10 ms 执行一次电机控制
uint32_t motor_now = HAL_GetTick();

if ((uint32_t)(motor_now - motor_last_tick) >= 10U)
{
    motor_last_tick = motor_now;
    Motor_ControlTick();
}
// 每隔约 100 ms 发送最新的 IMU、遥控器和电机状态
if ((uint32_t)(HAL_GetTick() - uart_last_tick) >= 100U &&
    huart1.gState == HAL_UART_STATE_READY)
{
    uart_last_tick = HAL_GetTick();

// 使用独立采样任务的最新读取结果
uint8_t imu_read_ok = imu_sample_ok;
    // 复制遥控器数据
    int16_t ch[4];
    uint8_t s1, s2, received;
    uint32_t last_tick;

    uint32_t irq_state = __get_PRIMASK();
    __disable_irq();

    for (uint8_t i = 0; i < 4U; i++)
    {
        ch[i] = rc_ch[i];
    }

    s1 = rc_s1;
    s2 = rc_s2;
    received = rc_received;
    last_tick = rc_last_tick;

    __set_PRIMASK(irq_state);

    uint8_t online = received &&
        (uint32_t)(HAL_GetTick() - last_tick) < 100U;

    // 断联时不显示旧的遥控器数值
    if (!online)
    {
        for (uint8_t i = 0; i < 4U; i++)
        {
            ch[i] = 0;
        }

        s1 = 0;
        s2 = 0;
    }

    int len;
    const char *att_state =
    imu_attitude_fault ? "ERR" :
    (imu_attitude_ready ? "OK" : "CAL");

int32_t yaw_mdeg = (int32_t)(imu_yaw_deg * 1000.0f);

    if (imu_read_ok)
    {
        // 乘 1000 后按整数输出，避免依赖 printf 浮点支持
        int32_t ax = (int32_t)(imu_ax * 1000.0f);
        int32_t ay = (int32_t)(imu_ay * 1000.0f);
        int32_t az = (int32_t)(imu_az * 1000.0f);

        int32_t gx = (int32_t)(imu_gx * 1000.0f);
        int32_t gy = (int32_t)(imu_gy * 1000.0f);
        int32_t gz = (int32_t)(imu_gz * 1000.0f);

        len = snprintf(
            uart_tx_buf, sizeof(uart_tx_buf),
           "ATT=%s YAW[mdeg]=%ld CAL=%u/200 "
           "ACC[mm/s2]=%ld,%ld,%ld GYR[mdeg/s]=%ld,%ld,%ld "
           "RC=%s CH=%d,%d,%d,%d SW=%u,%u\r\n",
           att_state,
           (long)yaw_mdeg,
           (unsigned)imu_cal_count,
            (long)ax, (long)ay, (long)az,
            (long)gx, (long)gy, (long)gz,
            online ? "ON" : "OFF",
            (int)ch[0], (int)ch[1], (int)ch[2], (int)ch[3],
            (unsigned)s1, (unsigned)s2);
    }
    else
    {
        len = snprintf(
            uart_tx_buf, sizeof(uart_tx_buf),
            "IMU=ERROR RC=%s CH=%d,%d,%d,%d SW=%u,%u\r\n",
            online ? "ON" : "OFF",
            (int)ch[0], (int)ch[1], (int)ch[2], (int)ch[3],
            (unsigned)s1, (unsigned)s2);
    }

    if (len > 0 && (size_t)len < sizeof(uart_tx_buf))
{
    size_t used = (size_t)len;
    uint8_t format_ok = 1U;

    // 暂时显示全部 7 个可能的 ID，便于现场确认电机编号
    for (uint8_t id = 1; id <= 7U; id++)
    {
        MotorFeedback fb;

        // 短暂关闭中断，取得同一帧的完整反馈
        uint32_t saved_irq = __get_PRIMASK();
        __disable_irq();
        fb = motor_fb[id];
        __set_PRIMASK(saved_irq);

        uint8_t motor_online =
            fb.received &&
            (uint32_t)(HAL_GetTick() - fb.last_rx_tick) < 100U;

        int added;

        if (motor_online)
{
    added = snprintf(
        uart_tx_buf + used,
        sizeof(uart_tx_buf) - used,
        "M%u=ON ENC=%u POS=%s CNT=%ld RPM=%d T=%u N=%lu\r\n",
        (unsigned)id,
        (unsigned)fb.encoder,
        fb.angle_valid ? "OK" : "ERR",
        (long)fb.total_encoder,
        (int)fb.speed_rpm,
        (unsigned)fb.temperature,
        (unsigned long)fb.rx_count);
}
        else
        {
            // 离线时不把旧角度和转速当成当前数据输出
            added = snprintf(
                uart_tx_buf + used,
                sizeof(uart_tx_buf) - used,
                "M%u=OFF N=%lu\r\n",
                (unsigned)id,
                (unsigned long)fb.rx_count);
        }

        // 检查格式化错误和缓冲区不足
        if (added < 0 ||
            (size_t)added >= sizeof(uart_tx_buf) - used)
        {
            format_ok = 0U;
            break;
        }

        used += (size_t)added;
    }

    // IMU、遥控器、电机数据合并后只发送一次
    if (format_ok)
    {
        HAL_UART_Transmit_IT(
            &huart1,
            (uint8_t *)uart_tx_buf,
            (uint16_t)used);
    }
}
}
  }
  

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart,
                               uint16_t Size)
{
    // 只处理遥控器使用的 USART3
    if (huart->Instance == USART3)
    {
        // 忽略半缓冲区事件
        if (HAL_UARTEx_GetRxEventType(huart) == HAL_UART_RXEVENT_HT)
        {
            return;
        }

        rc_rx_size = Size;
        rc_rx_count++;

        // 添加遥控器数据解析，
        // 只解析在空闲事件结束时收到的完整 18 字节帧
if (HAL_UARTEx_GetRxEventType(huart) == HAL_UART_RXEVENT_IDLE &&
    Size == 18U)
{
    int16_t ch[4];
    uint8_t s1;
    uint8_t s2;

    ch[0] = (int16_t)(
        (rc_rx_buf[0] | (rc_rx_buf[1] << 8)) & 0x07FF);

    ch[1] = (int16_t)(
        ((rc_rx_buf[1] >> 3) | (rc_rx_buf[2] << 5)) & 0x07FF);

    ch[2] = (int16_t)(
        ((rc_rx_buf[2] >> 6) | (rc_rx_buf[3] << 2) |
         (rc_rx_buf[4] << 10)) & 0x07FF);

    ch[3] = (int16_t)(
        ((rc_rx_buf[4] >> 1) | (rc_rx_buf[5] << 7)) & 0x07FF);

    s1 = (rc_rx_buf[5] >> 6) & 0x03;
    s2 = (rc_rx_buf[5] >> 4) & 0x03;

    // 原始中心值为 1024，减去后回中值接近 0
    uint8_t valid = 1;

    for (uint8_t i = 0; i < 4U; i++)
    {
        ch[i] -= 1024;

        // 正常范围约为 -660～660，留少量余量
        if (ch[i] < -700 || ch[i] > 700)
        {
            valid = 0;
        }
    }

    // 拨杆的有效编码为 1、2、3
    if (s1 == 0U || s2 == 0U)
    {
        valid = 0;
    }

    // 通过检查后，才更新供主循环使用的数据
    if (valid)
    {
        for (uint8_t i = 0; i < 4U; i++)
        {
            rc_ch[i] = ch[i];
        }

        rc_s1 = s1;
        rc_s2 = s2;
        rc_last_tick = HAL_GetTick();
        rc_received = 1;
    }
}
        
        // 必须在重新启动接收之前处理当前数据。

        // Normal 模式接收结束后，需要重新启动
        if (HAL_UARTEx_ReceiveToIdle_DMA(&huart3,
                                       rc_rx_buf,
                                       sizeof(rc_rx_buf)) == HAL_OK)
        {
            __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);
        }
    }
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        // 当前接收状态不再可信
        rc_received = 0;

        // 通知主循环恢复接收
        rc_need_restart = 1;
    }
}
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance != CAN1)
    {
        return;
    }

    CAN_RxHeaderTypeDef header;
    uint8_t data[8];

    // 每次中断最多取出 3 帧，避免长时间占用中断
    for (uint8_t n = 0; n < 3U; n++)
    {
        if (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) == 0U)
        {
            break;
        }

        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0,
                                &header, data) != HAL_OK)
        {
            break;
        }

        // 只解析 GM6020 的标准数据帧
        if (header.IDE != CAN_ID_STD ||
            header.RTR != CAN_RTR_DATA ||
            header.DLC != 8U ||
            header.StdId < 0x205U ||
            header.StdId > 0x20BU)
        {
            continue;
        }

        uint8_t id = (uint8_t)(header.StdId - 0x204U);
        uint16_t encoder =
            (uint16_t)(((uint16_t)data[0] << 8) | data[1]);

        if (encoder > 8191U)
        {
            continue;
        }

        uint32_t rx_now = HAL_GetTick();

if (!motor_fb[id].received)
{
    // 首次收到反馈：把当前位置作为累计角度零点
    motor_fb[id].total_encoder = 0;
    motor_fb[id].angle_valid = 1U;
}
else if ((uint32_t)(rx_now - motor_fb[id].last_rx_tick) >= 50U)
{
    // 中断接收期间可能转过未知圈数，不能继续猜测累计角度
    motor_fb[id].angle_valid = 0U;
}
else if (motor_fb[id].angle_valid)
{
    int32_t delta =
        (int32_t)encoder - (int32_t)motor_fb[id].encoder;

    // 处理单圈编码器边界
    if (delta > 4096)
    {
        delta -= 8192;
    }
    else if (delta < -4096)
    {
        delta += 8192;
    }

    motor_fb[id].total_encoder += delta;
}

motor_fb[id].encoder = encoder;
        motor_fb[id].speed_rpm =
            (int16_t)(((uint16_t)data[2] << 8) | data[3]);
        motor_fb[id].temperature = data[6];
        motor_fb[id].last_rx_tick = rx_now;
        motor_fb[id].rx_count++;
        motor_fb[id].received = 1U;
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
