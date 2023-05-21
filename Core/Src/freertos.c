/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <string.h>
#include "rcl/rcl.h"
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/int8.h>
#include <std_msgs/msg/int8_multi_array.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/u_int8_multi_array.h>
#include <rosidl_runtime_c/message_type_support_struct.h>

#include "usart.h"
#include "spi.h"
#include "i2c.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PUB 2
#define SUB 1
#define MODE SUB

#define BUF_SIZE 100
#define MSG_LEN 6

//Creating a structure to specify the SPI communication with MAX7301 will be 2 byte. One is the register address and the second is the data send to the MAX7301
typedef struct
{
    uint8_t address;
    uint8_t value;
} ConfigParam_t;

//Creating a structure to hold the state of each antenna
typedef struct
{
    uint8_t antEnable;
}ConfigAnt;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
//Declaration of an array with enable value for 2 daisy chained MAX7301
static const ConfigParam_t arrayConfig[] = {{.address = 0x04, .value = 0x01},
                                            {.address = 0x04, .value = 0x01}};

//Declaration of an array with the initialization parameters. Port P4 to P27 set has output.
static const ConfigParam_t arrayInitialize[] = {{.address = 0x09, .value = 0x55},
                                                {.address = 0x0A, .value = 0x55},
                                                {.address = 0x0B, .value = 0x55},
                                                {.address = 0x0C, .value = 0x55},
                                                {.address = 0x0D, .value = 0x55},
                                                {.address = 0x0E, .value = 0x55}};
//Declaration of each antenna state. Each bit in each byte is to enable one antenna.
//Ex : ConfigRISA[0] control antenna 1 to 8, ConfigRISA[1] control antenna 9 to 17
// ConfigRISA[0].antEnable = 0000 0001 -> enable antenna 1 				and disable all others from this byte
// ConfigRISA[3].antEnable = 1000 1001 -> enable antenna 27, 30 and 34	and disable all others from this byte
static ConfigAnt ConfigRISA[] = 	{{.antEnable = 0xAA},		// [0] Antenna 1 	to 8
                                  {.antEnable = 0xAA},		// [1] Antenna 9	to 16
                                  {.antEnable = 0xAA},		// [2] Antenna 17 	to 24
                                  {.antEnable = 0xAA},		// [3] Antenna 25 	to 32
                                  {.antEnable = 0xAA},		// [4] Antenna 33 	to 40
                                  {.antEnable = 0xAA}};		// [5] Antenna 41 	to 48

static ConfigAnt ConfigRISB[] = 	{{.antEnable = 0xAA},		// [0] Antenna 1 	to 8
                                  {.antEnable = 0xAA},		// [1] Antenna 9	to 16
                                  {.antEnable = 0xAA},		// [2] Antenna 17 	to 24
                                  {.antEnable = 0xAA},		// [3] Antenna 25 	to 32
                                  {.antEnable = 0xAA},		// [4] Antenna 33 	to 40
                                  {.antEnable = 0xAA}};		// [5] Antenna 41 	to 48
/* USER CODE END Variables */
/* Definitions for Enable */
osThreadId_t EnableHandle;
const osThreadAttr_t Enable_attributes = {
  .name = "Enable",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for Init */
osThreadId_t InitHandle;
const osThreadAttr_t Init_attributes = {
  .name = "Init",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for Test */
osThreadId_t TestHandle;
const osThreadAttr_t Test_attributes = {
  .name = "Test",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Update */
osThreadId_t UpdateHandle;
const osThreadAttr_t Update_attributes = {
  .name = "Update",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
uint32_t defaultTaskBuffer[ 3000 ];
osStaticThreadDef_t defaultTaskControlBlock;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .cb_mem = &defaultTaskControlBlock,
  .cb_size = sizeof(defaultTaskControlBlock),
  .stack_mem = &defaultTaskBuffer[0],
  .stack_size = sizeof(defaultTaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myBinarySem */
osSemaphoreId_t myBinarySemHandle;
const osSemaphoreAttr_t myBinarySem_attributes = {
  .name = "myBinarySem"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

void * microros_allocate(size_t size, void * state);
void microros_deallocate(void * pointer, void * state);
void * microros_reallocate(void * pointer, size_t size, void * state);
void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);

void read_cb(const void* msg);
/* USER CODE END FunctionPrototypes */

void StartEnable(void *argument);
void StartInit(void *argument);
void StartTest(void *argument);
void StartUpdate(void *argument);
void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of myBinarySem */
  myBinarySemHandle = osSemaphoreNew(1, 1, &myBinarySem_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Enable */
  EnableHandle = osThreadNew(StartEnable, NULL, &Enable_attributes);

  /* creation of Init */
  InitHandle = osThreadNew(StartInit, NULL, &Init_attributes);

  /* creation of Test */
#if 0
  TestHandle = osThreadNew(StartTest, NULL, &Test_attributes);
#endif

  /* creation of Update */
#if 0
  UpdateHandle = osThreadNew(StartUpdate, NULL, &Update_attributes);
#endif

  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartEnable */
/**
  * @brief  Function implementing the Enable thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartEnable */
void StartEnable(void *argument)
{
  /* USER CODE BEGIN StartEnable */
  // CS_LOW
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_RESET);
  // RISA
  HAL_SPI_Transmit(&hspi1, (uint8_t *)&arrayConfig[0].address, 1, 10);
  HAL_SPI_Transmit(&hspi1, (uint8_t *)&arrayConfig[0].value, 1, 10);
  HAL_SPI_Transmit(&hspi1, (uint8_t *)&arrayConfig[1].address, 1, 10);
  HAL_SPI_Transmit(&hspi1, (uint8_t *)&arrayConfig[1].value, 1, 10);
  // RISB
  HAL_SPI_Transmit(&hspi2, (uint8_t *)&arrayConfig[0].address, 1, 10);
  HAL_SPI_Transmit(&hspi2, (uint8_t *)&arrayConfig[0].value, 1, 10);
  HAL_SPI_Transmit(&hspi2, (uint8_t *)&arrayConfig[1].address, 1, 10);
  HAL_SPI_Transmit(&hspi2, (uint8_t *)&arrayConfig[1].value, 1, 10);
  // CS_HIGHT
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_SET);
  //uint8_t data[]= "Enable completed\n";
  //HAL_UART_Transmit(&huart3, data, sizeof(data),500);
  /* Give semaphore to Task 2 */
  osSemaphoreRelease(myBinarySemHandle);
  /* Delete Task 1 */
  vTaskDelete(EnableHandle);
  /* USER CODE END StartEnable */
}

/* USER CODE BEGIN Header_StartInit */
/**
* @brief Function implementing the Init thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartInit */
void StartInit(void *argument)
{
  /* USER CODE BEGIN StartInit */
  osSemaphoreAcquire(myBinarySemHandle, osWaitForever);
  uint8_t i = 0;
  //uint8_t data[]= "Init completed\n";
  for(i = 0; i < 6;i++)
  {
    // CS_LOW
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_RESET);
    // RISA
    HAL_SPI_Transmit(&hspi1, (uint8_t *)&arrayInitialize[i].address, 1, 10);
    HAL_SPI_Transmit(&hspi1, (uint8_t *)&arrayInitialize[i].value, 1, 10);
    HAL_SPI_Transmit(&hspi1, (uint8_t *)&arrayInitialize[i].address, 1, 10);
    HAL_SPI_Transmit(&hspi1, (uint8_t *)&arrayInitialize[i].value, 1, 10);
    // RISB
    HAL_SPI_Transmit(&hspi2, (uint8_t *)&arrayInitialize[i].address, 1, 10);
    HAL_SPI_Transmit(&hspi2, (uint8_t *)&arrayInitialize[i].value, 1, 10);
    HAL_SPI_Transmit(&hspi2, (uint8_t *)&arrayInitialize[i].address, 1, 10);
    HAL_SPI_Transmit(&hspi2, (uint8_t *)&arrayInitialize[i].value, 1, 10);
    // CS_HIGHT
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_SET);
    //HAL_UART_Transmit(&huart3, data, sizeof(data),500);
  }
  /* Give semaphore to Task 3 */
  osSemaphoreRelease(myBinarySemHandle);
  /* Delete Task 2 */
  vTaskDelete(InitHandle);
  /* USER CODE END StartInit */
}

/* USER CODE BEGIN Header_StartTest */
/**
* @brief Function implementing the Test thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTest */
void StartTest(void *argument)
{
  /* USER CODE BEGIN StartTest */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTest */
}

/* USER CODE BEGIN Header_StartUpdate */
/**
* @brief Function implementing the Update thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUpdate */
void StartUpdate(void *argument)
{
  /* USER CODE BEGIN StartUpdate */
  /* Infinite loop */
  for(;;)
  {
    osSemaphoreAcquire(myBinarySemHandle, osWaitForever);
    //This part Configure 8 antenna (P4 to P11 on each MAX7301)
    uint8_t bufferRISA1[2] = {0x44, ConfigRISA[0].antEnable};
    uint8_t bufferRISA2[2] = {0x44, ConfigRISA[3].antEnable};

    uint8_t bufferRISB1[2] = {0x44, ConfigRISA[0].antEnable};
    uint8_t bufferRISB2[2] = {0x44, ConfigRISA[3].antEnable};

    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_RESET);

    HAL_SPI_Transmit(&hspi1, (uint8_t *)&bufferRISA2, 2, 10);
    HAL_SPI_Transmit(&hspi1, (uint8_t *)&bufferRISA1, 2, 10);

    HAL_SPI_Transmit(&hspi2, (uint8_t *)&bufferRISB2, 2, 10);
    HAL_SPI_Transmit(&hspi2, (uint8_t *)&bufferRISB1, 2, 10);

    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_SET);


    //This part Configure 8 antenna (P12 to P19 on each MAX7301)
    bufferRISA1[0] = 0x4C;
    bufferRISA1[1] = ConfigRISA[1].antEnable;
    bufferRISA2[0] = 0x4C;
    bufferRISA2[1] = ConfigRISA[4].antEnable;

    bufferRISB1[0] = 0x4C;
    bufferRISB1[1] = ConfigRISB[1].antEnable;
    bufferRISB2[0] = 0x4C;
    bufferRISB2[1] = ConfigRISB[4].antEnable;

    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_RESET);

    HAL_SPI_Transmit(&hspi1, (uint8_t *)&bufferRISA2, 2, 10);
    HAL_SPI_Transmit(&hspi1, (uint8_t *)&bufferRISA1, 2, 10);

    HAL_SPI_Transmit(&hspi2, (uint8_t *)&bufferRISB2, 2, 10);
    HAL_SPI_Transmit(&hspi2, (uint8_t *)&bufferRISB1, 2, 10);

    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_SET);

    //This part Configure 8 antenna (P20 to P27 on each MAX7301)
    bufferRISA1[0] = 0x54;
    bufferRISA1[1] = ConfigRISA[2].antEnable;
    bufferRISA2[0] = 0x54;
    bufferRISA2[1] = ConfigRISA[5].antEnable;

    bufferRISB1[0] = 0x54;
    bufferRISB1[1] = ConfigRISB[2].antEnable;
    bufferRISB2[0] = 0x54;
    bufferRISB2[1] = ConfigRISB[5].antEnable;

    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_RESET);

    HAL_SPI_Transmit(&hspi1, (uint8_t *)&bufferRISA2, 2, 10);
    HAL_SPI_Transmit(&hspi1, (uint8_t *)&bufferRISA1, 2, 10);

    HAL_SPI_Transmit(&hspi2, (uint8_t *)&bufferRISB2, 2, 10);
    HAL_SPI_Transmit(&hspi2, (uint8_t *)&bufferRISB1, 2, 10);

    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_SET);

    //uint8_t data[]= "Update completed\n";
    //HAL_UART_Transmit(&huart3, data, sizeof(data),500);
    osDelay(500);
  }
  /* USER CODE END StartUpdate */
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
* @brief Function implementing the defaultTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  osSemaphoreAcquire(myBinarySemHandle, osWaitForever);

  // micro-ROS configuration
  rmw_uros_set_custom_transport(
          true,
          (void *) &huart3,
          cubemx_transport_open,
          cubemx_transport_close,
          cubemx_transport_write,
          cubemx_transport_read);

  rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
  freeRTOS_allocator.allocate = microros_allocate;
  freeRTOS_allocator.deallocate = microros_deallocate;
  freeRTOS_allocator.reallocate = microros_reallocate;
  freeRTOS_allocator.zero_allocate =  microros_zero_allocate;

  if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
    printf("Error on default allocators (line %d)\n", __LINE__);
  }

  // micro-ROS app
  rcl_ret_t ret = RCL_RET_OK;
  char err_log[BUF_SIZE] = "";
  // micro-ROS app

  //create init_options
#if 0
  std_msgs__msg__Int32 tx_msg = {.data = 66};
#endif

  uint8_t rx_data[MSG_LEN] = {0};
  std_msgs__msg__UInt8MultiArray rx_msg;
  rx_msg.data.data = rx_data;

  const rosidl_message_type_support_t* type_supp =  ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8MultiArray);
  const char* topic = "t1";

  rcl_allocator_t allocator = rcl_get_default_allocator();
  rclc_support_t support;
  ret = rclc_support_init(&support, 0, NULL, &allocator);
  if (RCL_RET_OK != ret) {strcat(err_log, "3"); }

#if (PUB == MODE)
  // create nodes
  rcl_node_t pub_node = rcl_get_zero_initialized_node();
  ret = rclc_node_init_default(&pub_node, "stm32_pub_node", "", &support);
  if (RCL_RET_OK != ret) {strcat(err_log, "4");}
#else
  rcl_node_t sub_node = rcl_get_zero_initialized_node();
  ret = rclc_node_init_default(&sub_node, "stm32_sub_node", "", &support);
  if (RCL_RET_OK != ret) {strcat(err_log, "5"); }
#endif

#if (PUB == MODE)
  // create publisher
  rcl_publisher_t pub = rcl_get_zero_initialized_publisher();
  ret = rclc_publisher_init_default(&pub, &pub_node, type_supp, topic);
  if (RCL_RET_OK != ret) {strcat(err_log, "6");}
#else
  // create subscriber
  rcl_subscription_t sub = rcl_get_zero_initialized_subscription();
  ret = rclc_subscription_init_default(&sub, &sub_node, type_supp, topic);
  if (RCL_RET_OK != ret) {strcat(err_log, "7"); }

  // alloc on ROS message Rx
  rmw_subscription_allocation_t allocation;

  // create executor
  rclc_executor_t exe = rclc_executor_get_zero_initialized_executor();
  rclc_executor_init(&exe, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&exe, &sub, &rx_msg, read_cb, ON_NEW_DATA);

  rclc_executor_spin(&exe);

#endif

  for(;;)
  {
#if (PUB == MODE)
#if 01
    ret = rcl_publish(&pub, &tx_msg, NULL);
    if (ret != RCL_RET_OK)
    {
      strcat(err_log, "8");
    }
#endif
#else
    #if 0
    ret = rcl_take(&sub, (void*)&rx_msg, NULL, &allocation);
    if (RCL_RET_OK != ret)
    {
      strncat(err_log, "9", BUF_SIZE);
      switch (ret)
      {
        case 1:
          HAL_GPIO_WritePin(GPIOB, LD1_Pin, 1);
          break;
        case 2:
          HAL_GPIO_WritePin(GPIOB, LD2_Pin, 1);
          break;
        case 3:
          HAL_GPIO_WritePin(GPIOB, LD3_Pin, 1);
          break;
        default:
          break;
      }

    }

    else
    {
      HAL_GPIO_WritePin(GPIOB, LD3_Pin, 1);
    }
#endif
#endif

    osDelay(10);
  }

  osSemaphoreRelease(myBinarySemHandle);
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void read_cb(const void* msg)
{
  uint8_t* rx_data = ((std_msgs__msg__UInt8MultiArray*)msg)->data.data;
#if 01
  if (1 == rx_data[0] && 2 == rx_data[1] && 3 == rx_data[2])
  {
    HAL_GPIO_WritePin(GPIOB, LD1_Pin, 1);
    HAL_GPIO_WritePin(GPIOB, LD2_Pin, 0);
    HAL_GPIO_WritePin(GPIOB, LD3_Pin, 0);
  }
  else if (2 == rx_data[0] && 2 == rx_data[1] && 3 == rx_data[2])
  {
    HAL_GPIO_WritePin(GPIOB, LD1_Pin, 0);
    HAL_GPIO_WritePin(GPIOB, LD2_Pin, 1);
    HAL_GPIO_WritePin(GPIOB, LD3_Pin, 0);
  }
  else if (69 == rx_data[3] && 200 == rx_data[4] && 255 == rx_data[5])
  {
    HAL_GPIO_WritePin(GPIOB, LD1_Pin, 0);
    HAL_GPIO_WritePin(GPIOB, LD2_Pin, 0);
    HAL_GPIO_WritePin(GPIOB, LD3_Pin, 1);
  }
  else
  {
    HAL_GPIO_WritePin(GPIOB, LD1_Pin, 0);
    HAL_GPIO_WritePin(GPIOB, LD2_Pin, 0);
    HAL_GPIO_WritePin(GPIOB, LD3_Pin, 0);
  }
#endif

  for (int i = 0; i < MSG_LEN; i++)
  {
    ConfigRISA[i].antEnable = rx_data[i];
  }

  //This part Configure 8 antenna (P4 to P11 on each MAX7301)
  uint8_t bufferRISA1[2] = {0x44, ConfigRISA[0].antEnable};
  uint8_t bufferRISA2[2] = {0x44, ConfigRISA[3].antEnable};

  uint8_t bufferRISB1[2] = {0x44, ConfigRISA[0].antEnable};
  uint8_t bufferRISB2[2] = {0x44, ConfigRISA[3].antEnable};

  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_RESET);

  HAL_SPI_Transmit(&hspi1, (uint8_t *)&bufferRISA2, 2, 10);
  HAL_SPI_Transmit(&hspi1, (uint8_t *)&bufferRISA1, 2, 10);

  HAL_SPI_Transmit(&hspi2, (uint8_t *)&bufferRISB2, 2, 10);
  HAL_SPI_Transmit(&hspi2, (uint8_t *)&bufferRISB1, 2, 10);

  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_SET);


  //This part Configure 8 antenna (P12 to P19 on each MAX7301)
  bufferRISA1[0] = 0x4C;
  bufferRISA1[1] = ConfigRISA[1].antEnable;
  bufferRISA2[0] = 0x4C;
  bufferRISA2[1] = ConfigRISA[4].antEnable;

  bufferRISB1[0] = 0x4C;
  bufferRISB1[1] = ConfigRISB[1].antEnable;
  bufferRISB2[0] = 0x4C;
  bufferRISB2[1] = ConfigRISB[4].antEnable;

  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_RESET);

  HAL_SPI_Transmit(&hspi1, (uint8_t *)&bufferRISA2, 2, 10);
  HAL_SPI_Transmit(&hspi1, (uint8_t *)&bufferRISA1, 2, 10);

  HAL_SPI_Transmit(&hspi2, (uint8_t *)&bufferRISB2, 2, 10);
  HAL_SPI_Transmit(&hspi2, (uint8_t *)&bufferRISB1, 2, 10);

  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_SET);

  //This part Configure 8 antenna (P20 to P27 on each MAX7301)
  bufferRISA1[0] = 0x54;
  bufferRISA1[1] = ConfigRISA[2].antEnable;
  bufferRISA2[0] = 0x54;
  bufferRISA2[1] = ConfigRISA[5].antEnable;

  bufferRISB1[0] = 0x54;
  bufferRISB1[1] = ConfigRISB[2].antEnable;
  bufferRISB2[0] = 0x54;
  bufferRISB2[1] = ConfigRISB[5].antEnable;

  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_RESET);

  HAL_SPI_Transmit(&hspi1, (uint8_t *)&bufferRISA2, 2, 10);
  HAL_SPI_Transmit(&hspi1, (uint8_t *)&bufferRISA1, 2, 10);

  HAL_SPI_Transmit(&hspi2, (uint8_t *)&bufferRISB2, 2, 10);
  HAL_SPI_Transmit(&hspi2, (uint8_t *)&bufferRISB1, 2, 10);

  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_SET);

  //uint8_t data[]= "Update completed\n";
  //HAL_UART_Transmit(&huart3, data, sizeof(data),500);
}
/* USER CODE END Application */

