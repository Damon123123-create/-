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
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// 发送期间保持内容不变
static char uart_tx_buf[160];
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
  /* USER CODE BEGIN 2 */
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
// 每隔 100 毫秒读取 IMU，并发送 IMU 和遥控器状态
if ((uint32_t)(HAL_GetTick() - uart_last_tick) >= 100U &&
    huart1.gState == HAL_UART_STATE_READY)
{
    uart_last_tick = HAL_GetTick();

    // 本次 IMU 读取是否成功
    uint8_t imu_read_ok = 0;

    if (imu_ready)
    {
        imu_read_ok = (BMI088_ReadData() == HAL_OK);
    }

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
            "ACC[mm/s2]=%ld,%ld,%ld GYR[mdeg/s]=%ld,%ld,%ld "
            "RC=%s CH=%d,%d,%d,%d SW=%u,%u\r\n",
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
        HAL_UART_Transmit_IT(
            &huart1, (uint8_t *)uart_tx_buf, (uint16_t)len);
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
