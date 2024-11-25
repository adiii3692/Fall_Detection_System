/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#define STEVAL_MKI109V3  /* little endian */
#define NUCLEO_F401RE    /* little endian */
//#define SPC584B_DIS      /* big endian */

/* ATTENTION: By default the driver is little endian. If you need switch
 *            to big endian please see "Endianness definitions" in the
 *            header file of the driver (_reg.h).
 */

#if defined(STEVAL_MKI109V3)
/* MKI109V3: Define communication interface */
#define SENSOR_BUS hspi2
/* MKI109V3: Vdd and Vddio power supply values */
#define PWM_3V3 915

#elif defined(NUCLEO_F401RE)
/* NUCLEO_F401RE: Define communication interface */
#define SENSOR_BUS hi2c1

#elif defined(SPC584B_DIS)
/* DISCOVERY_SPC584B: Define communication interface */
#define SENSOR_BUS I2CD1

#endif

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdio.h>
#include "lsm9ds1_reg.h"
#include <stdbool.h>

#if defined(NUCLEO_F401RE)
#include "stm32f4xx_hal.h"
#include "usart.h"
#include "gpio.h"
#include "i2c.h"

#elif defined(STEVAL_MKI109V3)
#include "stm32f4xx_hal.h"
#include "usbd_cdc_if.h"
#include "gpio.h"
#include "spi.h"
#include "tim.h"

#elif defined(SPC584B_DIS)
#include "components.h"
#endif

typedef struct {
  void   *hbus;
  uint8_t i2c_address;
  GPIO_TypeDef *cs_port;
  uint16_t cs_pin;
} sensbus_t;

/* Private macro -------------------------------------------------------------*/
#define    BOOT_TIME            20 //ms

/* Private variables ---------------------------------------------------------*/
#if defined(STEVAL_MKI109V3)
static sensbus_t imu_bus = {&SENSOR_BUS,
                            0,
                            CS_up_GPIO_Port,
                            CS_up_Pin
                           };
static sensbus_t mag_bus = {&SENSOR_BUS,
                            0,
                            CS_A_up_GPIO_Port,
                            CS_A_up_Pin
                           };
#elif defined(SPC584B_DIS) | defined(NUCLEO_F401RE)
static sensbus_t mag_bus = {&SENSOR_BUS,
                            LSM9DS1_MAG_I2C_ADD_H,
                            0,
                            0
                           };
static sensbus_t imu_bus = {&SENSOR_BUS,
                            LSM9DS1_IMU_I2C_ADD_H,
                            0,
                            0
                           };
#endif

static int16_t data_raw_acceleration[3];
static int16_t data_raw_angular_rate[3];
static int16_t data_raw_magnetic_field[3];
static float acceleration_mg[3];
static float angular_rate_mdps[3];
static float magnetic_field_mgauss[3];
static lsm9ds1_id_t whoamI;
static lsm9ds1_status_t reg;
static uint8_t rst;
static uint8_t tx_buffer[1000];

/* Extern variables ----------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
/*
 *   WARNING:
 *   Functions declare in this section are defined at the end of this file
 *   and are strictly related to the hardware platform used.
 *
 */
static int32_t platform_write_imu(void *handle, uint8_t reg,
                                  const uint8_t *bufp, uint16_t len);
static int32_t platform_read_imu(void *handle, uint8_t reg,
                                 uint8_t *bufp, uint16_t len);
static int32_t platform_write_mag(void *handle, uint8_t reg,
                                  const uint8_t *bufp, uint16_t len);
static int32_t platform_read_mag(void *handle, uint8_t reg,
                                 uint8_t *bufp, uint16_t len);
static void tx_com( uint8_t *tx_buffer, uint16_t len );
static void platform_delay(uint32_t ms);
static void platform_init(void);

/* Main Example --------------------------------------------------------------*/
void lsm9ds1_read_data_polling(void)
{
  stmdev_ctx_t dev_ctx_imu;
  stmdev_ctx_t dev_ctx_mag;
  /* Initialize inertial sensors (IMU) driver interface */
  dev_ctx_imu.write_reg = platform_write_imu;
  dev_ctx_imu.read_reg = platform_read_imu;
  dev_ctx_imu.handle = (void *)&imu_bus;
  /* Initialize magnetic sensors driver interface */
  dev_ctx_mag.write_reg = platform_write_mag;
  dev_ctx_mag.read_reg = platform_read_mag;
  dev_ctx_mag.handle = (void *)&mag_bus;
  /* Initialize platform specific hardware */
  platform_init();
  /* Wait sensor boot time */
  platform_delay(BOOT_TIME);
  /* Check device ID */
  lsm9ds1_dev_id_get(&dev_ctx_mag, &dev_ctx_imu, &whoamI);

  if (whoamI.imu != LSM9DS1_IMU_ID || whoamI.mag != LSM9DS1_MAG_ID) {
    while (1) {
      /* manage here device not found */
    }
  }

  /* Restore default configuration */
  lsm9ds1_dev_reset_set(&dev_ctx_mag, &dev_ctx_imu, PROPERTY_ENABLE);

  do {
    lsm9ds1_dev_reset_get(&dev_ctx_mag, &dev_ctx_imu, &rst);
  } while (rst);

  /* Enable Block Data Update */
  lsm9ds1_block_data_update_set(&dev_ctx_mag, &dev_ctx_imu,
                                PROPERTY_ENABLE);
  /* Set full scale */
  lsm9ds1_xl_full_scale_set(&dev_ctx_imu, LSM9DS1_4g);
  lsm9ds1_gy_full_scale_set(&dev_ctx_imu, LSM9DS1_2000dps);
  lsm9ds1_mag_full_scale_set(&dev_ctx_mag, LSM9DS1_16Ga);
  /* Configure filtering chain - See datasheet for filtering chain details */
  /* Accelerometer filtering chain */
  lsm9ds1_xl_filter_aalias_bandwidth_set(&dev_ctx_imu, LSM9DS1_AUTO);
  lsm9ds1_xl_filter_lp_bandwidth_set(&dev_ctx_imu,
                                     LSM9DS1_LP_ODR_DIV_50);
  lsm9ds1_xl_filter_out_path_set(&dev_ctx_imu, LSM9DS1_LP_OUT);
  /* Gyroscope filtering chain */
  lsm9ds1_gy_filter_lp_bandwidth_set(&dev_ctx_imu,
                                     LSM9DS1_LP_ULTRA_LIGHT);
  lsm9ds1_gy_filter_hp_bandwidth_set(&dev_ctx_imu, LSM9DS1_HP_MEDIUM);
  lsm9ds1_gy_filter_out_path_set(&dev_ctx_imu,
                                 LSM9DS1_LPF1_HPF_LPF2_OUT);
  /* Set Output Data Rate / Power mode */
  lsm9ds1_imu_data_rate_set(&dev_ctx_imu, LSM9DS1_IMU_59Hz5);
  lsm9ds1_mag_data_rate_set(&dev_ctx_mag, LSM9DS1_MAG_UHP_10Hz);

  /* Read samples in polling mode (no int) */
  while (1) {
    /* Read device status register */
    lsm9ds1_dev_status_get(&dev_ctx_mag, &dev_ctx_imu, &reg);

    if ( reg.status_imu.xlda && reg.status_imu.gda ) {
      /* Read imu data */
      memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
      memset(data_raw_angular_rate, 0x00, 3 * sizeof(int16_t));
      lsm9ds1_acceleration_raw_get(&dev_ctx_imu,
                                   data_raw_acceleration);
      lsm9ds1_angular_rate_raw_get(&dev_ctx_imu,
                                   data_raw_angular_rate);
      acceleration_mg[0] = lsm9ds1_from_fs4g_to_mg(
                             data_raw_acceleration[0]);
      acceleration_mg[1] = lsm9ds1_from_fs4g_to_mg(
                             data_raw_acceleration[1]);
      acceleration_mg[2] = lsm9ds1_from_fs4g_to_mg(
                             data_raw_acceleration[2]);
      angular_rate_mdps[0] = lsm9ds1_from_fs2000dps_to_mdps(
                               data_raw_angular_rate[0]);
      angular_rate_mdps[1] = lsm9ds1_from_fs2000dps_to_mdps(
                               data_raw_angular_rate[1]);
      angular_rate_mdps[2] = lsm9ds1_from_fs2000dps_to_mdps(
                               data_raw_angular_rate[2]);
      snprintf((char *)tx_buffer, sizeof(tx_buffer),
              "IMU - [mg]:%4.2f\t%4.2f\t%4.2f\t[mdps]:%4.2f\t%4.2f\t%4.2f\r\n",
              acceleration_mg[0], acceleration_mg[1], acceleration_mg[2],
              angular_rate_mdps[0], angular_rate_mdps[1], angular_rate_mdps[2]);
      tx_com(tx_buffer, strlen((char const *)tx_buffer));
    }

    if ( reg.status_mag.zyxda ) {
      /* Read magnetometer data */
      memset(data_raw_magnetic_field, 0x00, 3 * sizeof(int16_t));
      lsm9ds1_magnetic_raw_get(&dev_ctx_mag, data_raw_magnetic_field);
      magnetic_field_mgauss[0] = lsm9ds1_from_fs16gauss_to_mG(
                                   data_raw_magnetic_field[0]);
      magnetic_field_mgauss[1] = lsm9ds1_from_fs16gauss_to_mG(
                                   data_raw_magnetic_field[1]);
      magnetic_field_mgauss[2] = lsm9ds1_from_fs16gauss_to_mG(
                                   data_raw_magnetic_field[2]);
      snprintf((char *)tx_buffer, sizeof(tx_buffer), "MAG - [mG]:%4.2f\t%4.2f\t%4.2f\r\n",
              magnetic_field_mgauss[0], magnetic_field_mgauss[1],
              magnetic_field_mgauss[2]);
      tx_com(tx_buffer, strlen((char const *)tx_buffer));
    }
  }
}

/*
 * @brief  Write generic imu register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
static int32_t platform_write_imu(void *handle, uint8_t reg,
                                  const uint8_t *bufp, uint16_t len)
{
  sensbus_t *sensbus = (sensbus_t *)handle;
#if defined(NUCLEO_F401RE)
  HAL_I2C_Mem_Write(sensbus->hbus, sensbus->i2c_address, reg,
                    I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000);
#elif defined(STEVAL_MKI109V3)
  HAL_GPIO_WritePin(sensbus->cs_port, sensbus->cs_pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(sensbus->hbus, &reg, 1, 1000);
  HAL_SPI_Transmit(sensbus->hbus, (uint8_t*) bufp, len, 1000);
  HAL_GPIO_WritePin(sensbus->cs_port, sensbus->cs_pin, GPIO_PIN_SET);
#elif defined(SPC584B_DIS)
  i2c_lld_write(sensbus->hbus,  sensbus->i2c_address & 0xFE, reg,
                (uint8_t*) bufp, len);
#endif
  return 0;
}

/*
 * @brief  Write generic magnetometer register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
static int32_t platform_write_mag(void *handle, uint8_t reg,
                                  const uint8_t *bufp, uint16_t len)
{
  sensbus_t *sensbus = (sensbus_t *)handle;
#if defined(NUCLEO_F401RE)
  /* Write multiple command */
  reg |= 0x80;
  HAL_I2C_Mem_Write(sensbus->hbus, sensbus->i2c_address, reg,
                    I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000);
#elif defined(STEVAL_MKI109V3)
  /* Write multiple command */
  reg |= 0x40;
  HAL_GPIO_WritePin(sensbus->cs_port, sensbus->cs_pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(sensbus->hbus, &reg, 1, 1000);
  HAL_SPI_Transmit(sensbus->hbus, (uint8_t*) bufp, len, 1000);
  HAL_GPIO_WritePin(sensbus->cs_port, sensbus->cs_pin, GPIO_PIN_SET);
#elif defined(SPC584B_DIS)
  /* Write multiple command */
  reg |= 0x80;
  i2c_lld_write(sensbus->hbus, sensbus->i2c_address & 0xFE, reg,
                (uint8_t*) bufp, len);
#endif
  return 0;
}

/*
 * @brief  Read generic imu register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t platform_read_imu(void *handle, uint8_t reg,
                                 uint8_t *bufp, uint16_t len)
{
  sensbus_t *sensbus = (sensbus_t *)handle;
#if defined(NUCLEO_F401RE)
  HAL_I2C_Mem_Read(sensbus->hbus, sensbus->i2c_address, reg,
                   I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
#elif defined(STEVAL_MKI109V3)
  /* Read command */
  reg |= 0x80;
  HAL_GPIO_WritePin(sensbus->cs_port, sensbus->cs_pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(sensbus->hbus, &reg, 1, 1000);
  HAL_SPI_Receive(sensbus->hbus, bufp, len, 1000);
  HAL_GPIO_WritePin(sensbus->cs_port, sensbus->cs_pin, GPIO_PIN_SET);
#elif defined(SPC584B_DIS)
  i2c_lld_read(sensbus->hbus, sensbus->i2c_address & 0xFE, reg, bufp,
               len);
#endif
  return 0;
}

/*
 * @brief  Read generic magnetometer register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t platform_read_mag(void *handle, uint8_t reg,
                                 uint8_t *bufp, uint16_t len)
{
  sensbus_t *sensbus = (sensbus_t *)handle;
#if defined(NUCLEO_F401RE)
  /* Read multiple command */
  reg |= 0x80;
  HAL_I2C_Mem_Read(sensbus->hbus, sensbus->i2c_address, reg,
                   I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
#elif defined(STEVAL_MKI109V3)
  /* Read multiple command */
  reg |= 0xC0;
  HAL_GPIO_WritePin(sensbus->cs_port, sensbus->cs_pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(sensbus->hbus, &reg, 1, 1000);
  HAL_SPI_Receive(sensbus->hbus, bufp, len, 1000);
  HAL_GPIO_WritePin(sensbus->cs_port, sensbus->cs_pin, GPIO_PIN_SET);
#elif defined(SPC584B_DIS)
  /* Read multiple command */
  reg |= 0x80;
  i2c_lld_read(sensbus->hbus, sensbus->i2c_address & 0xFE, reg, bufp,
               len);
#endif
  return 0;
}

/*
 * @brief  Send buffer to console (platform dependent)
 *
 * @param  tx_buffer     buffer to transmit
 * @param  len           number of byte to send
 *
 */
static void tx_com(uint8_t *tx_buffer, uint16_t len)
{
#if defined(NUCLEO_F401RE)
  HAL_UART_Transmit(&huart2, tx_buffer, len, 1000);
#elif defined(STEVAL_MKI109V3)
  CDC_Transmit_FS(tx_buffer, len);
#elif defined(SPC584B_DIS)
  sd_lld_write(&SD2, tx_buffer, len);
#endif
}

/*
 * @brief  platform specific delay (platform dependent)
 *
 * @param  ms        delay in ms
 *
 */
static void platform_delay(uint32_t ms)
{
#if defined(NUCLEO_F401RE) | defined(STEVAL_MKI109V3)
  HAL_Delay(ms);
#elif defined(SPC584B_DIS)
  osalThreadDelayMilliseconds(ms);
#endif
}

/*
 * @brief  platform specific initialization (platform dependent)
 */
static void platform_init(void)
{
#if defined(STEVAL_MKI109V3)
  TIM3->CCR1 = PWM_3V3;
  TIM3->CCR2 = PWM_3V3;
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_Delay(1000);
#endif
}
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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t TxData[8] = {0};
volatile uint8_t transmissionComplete = 1;

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart1) {
        transmissionComplete = 1;
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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();

  stmdev_ctx_t dev_ctx_imu;
    stmdev_ctx_t dev_ctx_mag;
    /* Initialize inertial sensors (IMU) driver interface */
    dev_ctx_imu.write_reg = platform_write_imu;
    dev_ctx_imu.read_reg = platform_read_imu;
    dev_ctx_imu.handle = (void *)&imu_bus;
    /* Initialize magnetic sensors driver interface */
    dev_ctx_mag.write_reg = platform_write_mag;
    dev_ctx_mag.read_reg = platform_read_mag;
    dev_ctx_mag.handle = (void *)&mag_bus;
    /* Initialize platform specific hardware */
    platform_init();
    /* Wait sensor boot time */
    platform_delay(BOOT_TIME);
    /* Check device ID */
    lsm9ds1_dev_id_get(&dev_ctx_mag, &dev_ctx_imu, &whoamI);

    if (whoamI.imu != LSM9DS1_IMU_ID || whoamI.mag != LSM9DS1_MAG_ID) {
      while (1) {
        /* manage here device not found */
      }
    }

    /* Restore default configuration */
    lsm9ds1_dev_reset_set(&dev_ctx_mag, &dev_ctx_imu, PROPERTY_ENABLE);

    do {
      lsm9ds1_dev_reset_get(&dev_ctx_mag, &dev_ctx_imu, &rst);
    } while (rst);

    /* Enable Block Data Update */
    lsm9ds1_block_data_update_set(&dev_ctx_mag, &dev_ctx_imu,
                                  PROPERTY_ENABLE);
    /* Set full scale */
    lsm9ds1_xl_full_scale_set(&dev_ctx_imu, LSM9DS1_4g);
    lsm9ds1_gy_full_scale_set(&dev_ctx_imu, LSM9DS1_2000dps);
    lsm9ds1_mag_full_scale_set(&dev_ctx_mag, LSM9DS1_16Ga);
    /* Configure filtering chain - See datasheet for filtering chain details */
    /* Accelerometer filtering chain */
    lsm9ds1_xl_filter_aalias_bandwidth_set(&dev_ctx_imu, LSM9DS1_AUTO);
    lsm9ds1_xl_filter_lp_bandwidth_set(&dev_ctx_imu,
                                       LSM9DS1_LP_ODR_DIV_50);
    lsm9ds1_xl_filter_out_path_set(&dev_ctx_imu, LSM9DS1_LP_OUT);
    /* Gyroscope filtering chain */
    lsm9ds1_gy_filter_lp_bandwidth_set(&dev_ctx_imu,
                                       LSM9DS1_LP_ULTRA_LIGHT);
    lsm9ds1_gy_filter_hp_bandwidth_set(&dev_ctx_imu, LSM9DS1_HP_MEDIUM);
    lsm9ds1_gy_filter_out_path_set(&dev_ctx_imu,
                                   LSM9DS1_LPF1_HPF_LPF2_OUT);
    /* Set Output Data Rate / Power mode */
    lsm9ds1_imu_data_rate_set(&dev_ctx_imu, LSM9DS1_IMU_59Hz5);
    lsm9ds1_mag_data_rate_set(&dev_ctx_mag, LSM9DS1_MAG_UHP_10Hz);

    /* Read samples in polling mode (no int) */
    while (1) {
      /* Read device status register */
      lsm9ds1_dev_status_get(&dev_ctx_mag, &dev_ctx_imu, &reg);

      if ( reg.status_imu.xlda && reg.status_imu.gda ) {
        /* Read imu data */
        memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
        memset(data_raw_angular_rate, 0x00, 3 * sizeof(int16_t));
        lsm9ds1_acceleration_raw_get(&dev_ctx_imu,
                                     data_raw_acceleration);
        lsm9ds1_angular_rate_raw_get(&dev_ctx_imu,
                                     data_raw_angular_rate);
        acceleration_mg[0] = lsm9ds1_from_fs4g_to_mg(
                               data_raw_acceleration[0]);
        acceleration_mg[1] = lsm9ds1_from_fs4g_to_mg(
                               data_raw_acceleration[1]);
        acceleration_mg[2] = lsm9ds1_from_fs4g_to_mg(
                               data_raw_acceleration[2]);
        angular_rate_mdps[0] = lsm9ds1_from_fs2000dps_to_mdps(
                                 data_raw_angular_rate[0]);
        angular_rate_mdps[1] = lsm9ds1_from_fs2000dps_to_mdps(
                                 data_raw_angular_rate[1]);
        angular_rate_mdps[2] = lsm9ds1_from_fs2000dps_to_mdps(
                                 data_raw_angular_rate[2]);
        snprintf((char *)tx_buffer, sizeof(tx_buffer),
                "IMU - [mg]:%4.2f\t%4.2f\t%4.2f\t[mdps]:%4.2f\t%4.2f\t%4.2f\r\n",
                acceleration_mg[0], acceleration_mg[1], acceleration_mg[2],
                angular_rate_mdps[0], angular_rate_mdps[1], angular_rate_mdps[2]);
        tx_com(tx_buffer, strlen((char const *)tx_buffer));
        HAL_Delay(1000);
        if ((acceleration_mg[0]<0) || (acceleration_mg[1]<0) || (acceleration_mg[2]<0)){
        	TxData[0] = 1;
        }
        else{
        	TxData[0] = 0;
        }
        HAL_HalfDuplex_EnableTransmitter(&huart1);
        HAL_UART_Transmit_IT(&huart1, TxData, sizeof(TxData));
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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

#ifdef  USE_FULL_ASSERT
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
