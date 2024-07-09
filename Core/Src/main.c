
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include <math.h>
#include <string.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define MAX_POINTS 450 // 最大点数
#define CIRCLE_DIAMETER 200.0f // 圆的直径
#define CIRCLE_RADIUS (CIRCLE_DIAMETER / 2.0f)	// 圆的半径
#define M_PI 3.14159265358979
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
  float x;  // X坐标
  float y;  // Y坐标
} Point;

typedef struct {
  float xx; // 扩展点的X坐标
  float yy; // 扩展点的Y坐标
} Extend;

typedef struct {
  uint16_t distance;  // 距离 (mm)
  uint8_t intensity;  // 强度
  float angle;        // 角度 (度)
} LidarData;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint16_t pointCount = 0;  // 当前点数
Point pointCloud[MAX_POINTS];
Extend pointExtend[MAX_POINTS];
uint8_t rxBuffer[200] = {0}; // 接收缓冲区
uint8_t rxData[58] = {0}; // 处理数据缓冲区
float bestAngle = 0.0f; // 最佳角度
LidarData lidarData[MAX_POINTS];  // 激光雷达数据
uint16_t lidarDataCount = 0;      // 激光雷达数据计数

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
void processLidarFrame(uint8_t *frame)
{
    // 校验帧头，帧头应为 0xA5 和 0x5A
    if (frame[0] != 0xA5 || frame[1] != 0x5A) {
        // 帧头不正确，忽略此帧
        return;
    }

    // 获取帧长度，长度保存在 Byte_2 和 Byte_3 中
    uint16_t frameLength = (frame[2] << 8) | frame[3];

    // 校验帧长度，帧长度应为 58 字节
    if (frameLength != 58) {
        // 帧长度不正确，忽略此帧
        return;
    }

    // 获取速度，保存在 Byte_4 和 Byte_5 中
    uint16_t speed = (frame[4] << 8) | frame[5];
    // 此处可以处理速度信息

    // 获取起始角度，保存在 Byte_6 和 Byte_7 中
    uint16_t startAngle = (frame[6] << 8) | frame[7];
    float startAngleDeg = startAngle / 100.0f;  // 转换为度

    // 获取结束角度，保存在 Byte_54 和 Byte_55 中
    uint16_t endAngle = (frame[54] << 8) | frame[55];
    float endAngleDeg = endAngle / 100.0f;  // 转换为度

    // 获取点云数据，保存在 Byte_8 到 Byte_53 中
    for (int i = 0; i < 50; ++i) {
        // 每个点由 2 字节距离和 1 字节强度组成
        uint16_t distance = (frame[8 + i * 3] << 8) | frame[9 + i * 3];
        uint8_t intensity = frame[10 + i * 3];

        // 计算当前点的角度
        float currentAngle = startAngleDeg + i * ((endAngleDeg - startAngleDeg) / 50.0f);

        // 保存激光雷达数据
        lidarData[lidarDataCount].distance = distance;
        lidarData[lidarDataCount].intensity = intensity;
        lidarData[lidarDataCount].angle = currentAngle;

        // 防止数组越界
        if (lidarDataCount < MAX_POINTS - 1) {
            lidarDataCount++;
        } else {
            // 超过最大点数，停止处理
            break;
        }
    }
}

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void processLidarData(uint8_t *data);
float distance(Extend a, Point b);
void detectCircles(Point *points, Extend *Epoints, float *bestAngle);
Point extendPoint(Point p, float radius);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    HAL_UART_Receive_DMA(&huart5, rxBuffer, sizeof(rxBuffer)); // 重新启动DMA接收

    for(int i = 0; i < sizeof(rxBuffer) - 58; i++)
    {
        if(rxBuffer[i] == 0xA5 && rxBuffer[i+1] == 0x5A)
        {
            memcpy(rxData, &rxBuffer[i], 58); // 拷贝帧数据到处理缓冲区
            processLidarData(rxData); // 处理激光雷达数据
            processLidarFrame(rxData); // 处理激光雷达帧数据
            break; // 处理完一帧数据后退出
        }
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
  float detectedAngle = 0.0f; // 记录检测到的最佳角度
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
  MX_TIM1_Init();
  MX_TIM12_Init();
  MX_UART5_Init();
  /* USER CODE BEGIN 2 */
  // 发送启动指令
  uint8_t start_cmd[] = {
    0xA5, 0x5A, 0x55, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0xFA, 0xFB
  };
  HAL_UART_Transmit_DMA(&huart5, start_cmd, sizeof(start_cmd));
  HAL_Delay(10);
  HAL_UART_Receive_DMA(&huart5, rxBuffer, sizeof(rxBuffer)); // 启动DMA接收
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
      // 检测圆
      detectCircles(pointCloud, pointExtend, &bestAngle);
      // 处理其他逻辑
  }
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
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

 */待完善*/

