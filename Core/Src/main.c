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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "math.h"
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* HMC5883L data structure */
typedef struct {
    I2C_HandleTypeDef *i2c_handle;
    uint8_t addr;
    int16_t x, y, z;
    int16_t mag_strength;
} HMC5883L_HandleTypeDef;

/* BME280 data structure */
typedef struct {
    I2C_HandleTypeDef *i2c_handle;
    uint8_t addr;
    uint16_t dig_T1, dig_T2, dig_T3;
    uint16_t dig_P1, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
    uint8_t dig_H1, dig_H2, dig_H3, dig_H4, dig_H5, dig_H6;
    int32_t t_fine; // Store fine temperature value
} BME280_HandleTypeDef;

/* BH1750 data structure */
typedef struct {
    I2C_HandleTypeDef *i2c_handle;
    uint8_t addr;
} BH1750_HandleTypeDef;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BME280_ADDR 0x76
#define BH1750_ADDR 0x23
#define LCD_ADDR 0x27
#define LCD_ROWS 2
#define LCD_COLS 16
#define HMC5883L_ADDR 0x1E


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* BME280 register definitions */
#define BME280_REG_ID 0xD0
#define BME280_REG_CTRL_HUM 0xF2
#define BME280_REG_CTRL_MEAS 0xF4
#define BME280_REG_CONFIG 0xF5
#define BME280_REG_PRESS_MSB 0xF7
#define BME280_REG_TEMP_MSB 0xFA
#define BME280_REG_HUM_MSB 0xFD

/* BME280 calibration registers */
#define BME280_REG_TEMP_CALIB 0x88
#define BME280_REG_PRESS_CALIB 0x8E
#define BME280_REG_HUM_CALIB 0xE1

/* HMC5883L register definitions */
#define HMC5883L_REG_CONFIG_A 0x00
#define HMC5883L_REG_CONFIG_B 0x01
#define HMC5883L_REG_MODE 0x02
#define HMC5883L_REG_DATA_X_MSB 0x03
#define HMC5883L_REG_STATUS 0x09

/* LCD commands */
#define LCD_CLEAR 0x01
#define LCD_ENTRY_MODE 0x06
#define LCD_DISPLAY_ON 0x0C
#define LCD_FUNCTION_SET 0x28
#define LCD_HOME 0x02
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */
BME280_HandleTypeDef bme280;
BH1750_HandleTypeDef bh1750;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* LCD functions */
void lcd_write_nibble(uint8_t nibble, uint8_t rs) {
    uint8_t data = (nibble << 4) | (rs << 0) | 0x08; // Ensure backlight is always ON
    HAL_I2C_Master_Transmit(&hi2c1, LCD_ADDR << 1, &data, 1, 100);
    data |= 0x04;  // Enable high
    HAL_I2C_Master_Transmit(&hi2c1, LCD_ADDR << 1, &data, 1, 100);
    data &= ~0x04; // Enable low
    HAL_I2C_Master_Transmit(&hi2c1, LCD_ADDR << 1, &data, 1, 100);
}

void lcd_send_cmd(uint8_t cmd) {
    lcd_write_nibble((cmd >> 4) & 0x0F, 0);
    lcd_write_nibble(cmd & 0x0F, 0);
    HAL_Delay(2);
}

void lcd_send_data(uint8_t data) {
    lcd_write_nibble((data >> 4) & 0x0F, 1);
    lcd_write_nibble(data & 0x0F, 1);
    HAL_Delay(2);
}

void lcd_init(void) {
    HAL_Delay(50); // Power-on delay
    lcd_send_cmd(0x03);
    HAL_Delay(5);
    lcd_send_cmd(0x03);
    HAL_Delay(1);
    lcd_send_cmd(0x03);
    lcd_send_cmd(0x02); // 4-bit mode
    lcd_send_cmd(LCD_FUNCTION_SET);
    lcd_send_cmd(LCD_DISPLAY_ON);
    lcd_send_cmd(LCD_CLEAR);
    lcd_send_cmd(LCD_ENTRY_MODE);
}

void lcd_write_string(char *str) {
    while (*str) {
        lcd_send_data(*str++);
    }
}

void lcd_set_cursor(uint8_t row, uint8_t column) {
    uint8_t pos = (row == 0) ? 0x80 + column : 0xC0 + column;
    lcd_send_cmd(pos);
}

void lcd_clear(void) {
    lcd_send_cmd(LCD_CLEAR);
    HAL_Delay(2);
}

void lcd_backlight(uint8_t state) {
    uint8_t data = state ? 0x08 : 0x00;
    HAL_I2C_Master_Transmit(&hi2c1, LCD_ADDR << 1, &data, 1, 100);
}




/* BME280 functions */
HAL_StatusTypeDef BME280_Read(uint8_t reg, uint8_t *data, uint16_t len) {
    return HAL_I2C_Mem_Read(&hi2c1, (bme280.addr << 1), reg, 1, data, len, HAL_MAX_DELAY);
}

HAL_StatusTypeDef BME280_ReadCalibrationData(BME280_HandleTypeDef *bme280) {
    uint8_t data[24];
    if (BME280_Read(BME280_REG_TEMP_CALIB, data, 24) != HAL_OK) {
        return HAL_ERROR;
    }

    bme280->dig_T1 = (data[1] << 8) | data[0];
    bme280->dig_T2 = (data[3] << 8) | data[2];
    bme280->dig_T3 = (data[5] << 8) | data[4];
    // Continue reading and saving other calibration values for pressure and humidity
    // You would similarly fetch pressure and humidity calibration data
    return HAL_OK;
}


HAL_StatusTypeDef BME280_Init(BME280_HandleTypeDef *bme280, I2C_HandleTypeDef *hi2c, uint8_t addr) {
    bme280->i2c_handle = hi2c;
    bme280->addr = addr;

    uint8_t id;
    if (BME280_Read(BME280_REG_ID, &id, 1) != HAL_OK || id != 0x60) {
        return HAL_ERROR; // ID mismatch
    }

    // Read calibration data
    if (BME280_ReadCalibrationData(bme280) != HAL_OK) {
        return HAL_ERROR;
    }

    uint8_t config[2];
    config[0] = BME280_REG_CTRL_HUM;
    config[1] = 0x01; // Humidity oversampling x1
    if (HAL_I2C_Master_Transmit(hi2c, addr << 1, config, 2, HAL_MAX_DELAY) != HAL_OK) {
        return HAL_ERROR;
    }

    config[0] = BME280_REG_CTRL_MEAS;
    config[1] = 0x27; // Temp and pressure oversampling x1, mode normal
    if (HAL_I2C_Master_Transmit(hi2c, addr << 1, config, 2, HAL_MAX_DELAY) != HAL_OK) {
        return HAL_ERROR;
    }

    return HAL_OK;
}

HAL_StatusTypeDef BME280_ReadData(BME280_HandleTypeDef *bme280, int *temperature, int *humidity, int *pressure) {
    uint8_t data[8];
    if (BME280_Read(BME280_REG_PRESS_MSB, data, 8) != HAL_OK) {
        return HAL_ERROR;
    }

    int adc_T = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4);
    int adc_P = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4);
    int adc_H = (data[6] << 8) | data[7];

    // Calculate temperature (using the BME280's formula and calibration data)
    int32_t var1 = (((adc_T >> 3) - (bme280->dig_T1 << 1)) * bme280->dig_T2) >> 11;
    int32_t var2 = (((((adc_T >> 4) - bme280->dig_T1) * ((adc_T >> 4) - bme280->dig_T1)) >> 12) * bme280->dig_T3) >> 14;
    bme280->t_fine = var1 + var2;
    *temperature = (bme280->t_fine * 5 + 128) >> 8; // Convert fine temperature to Celsius

    *pressure = adc_P / 256; // Adjust scaling as per calibration
    *humidity = adc_H / 1024; // Adjust scaling as per calibration

    return HAL_OK;
}

/* BH1750 functions */
HAL_StatusTypeDef BH1750_Init(BH1750_HandleTypeDef *bh1750, I2C_HandleTypeDef *hi2c, uint8_t addr) {
    bh1750->i2c_handle = hi2c;
    bh1750->addr = addr;

    uint8_t cmd = 0x01; // Power on the sensor
    if (HAL_I2C_Master_Transmit(hi2c, (bh1750->addr << 1), &cmd, 1, HAL_MAX_DELAY) != HAL_OK) {
        return HAL_ERROR;
    }
    HAL_Delay(100);

    cmd = 0x10; // Start measurement in high resolution mode
    if (HAL_I2C_Master_Transmit(hi2c, (bh1750->addr << 1), &cmd, 1, HAL_MAX_DELAY) != HAL_OK) {
        return HAL_ERROR;
    }

    return HAL_OK;
}

HAL_StatusTypeDef BH1750_ReadLightIntensity(BH1750_HandleTypeDef *bh1750, uint16_t *lux) {
    uint8_t data[2];
    if (HAL_I2C_Master_Receive(bh1750->i2c_handle, (bh1750->addr << 1), data, 2, HAL_MAX_DELAY) != HAL_OK) {
        return HAL_ERROR;
    }

    *lux = (data[0] << 8) | data[1];
    return HAL_OK;
}

/* HMC5883L Init function */
HAL_StatusTypeDef HMC5883L_Init(HMC5883L_HandleTypeDef *hmc5883l, I2C_HandleTypeDef *hi2c, uint8_t addr) {
    hmc5883l->i2c_handle = hi2c;
    hmc5883l->addr = addr;

    // Set configuration registers (default values for continuous measurement mode)
    uint8_t config_a[2] = { HMC5883L_REG_CONFIG_A, 0x70 }; // 8-average, 15 Hz, normal measurement
    HAL_I2C_Master_Transmit(hi2c, (hmc5883l->addr << 1), config_a, 2, HAL_MAX_DELAY);

    uint8_t config_b[2] = { HMC5883L_REG_CONFIG_B, 0x20 }; // Gain = 5 (default)
    HAL_I2C_Master_Transmit(hi2c, (hmc5883l->addr << 1), config_b, 2, HAL_MAX_DELAY);

    uint8_t mode[2] = { HMC5883L_REG_MODE, 0x00 }; // Continuous measurement mode
    HAL_I2C_Master_Transmit(hi2c, (hmc5883l->addr << 1), mode, 2, HAL_MAX_DELAY);

    return HAL_OK;
}

/* HMC5883L Read Data Function */
HAL_StatusTypeDef HMC5883L_ReadData(HMC5883L_HandleTypeDef *hmc5883l) {
    uint8_t data[6];

    // Receive 6 bytes of data (X, Z, Y components)
    if (HAL_I2C_Master_Receive(hmc5883l->i2c_handle, (hmc5883l->addr << 1), data, 6, HAL_MAX_DELAY) != HAL_OK) {
        return HAL_ERROR;
    }

    // Combine the data into 16-bit signed integers (MSB first)
    hmc5883l->x = (int16_t)((data[0] << 8) | data[1]);
    hmc5883l->z = (int16_t)((data[2] << 8) | data[3]);
    hmc5883l->y = (int16_t)((data[4] << 8) | data[5]);

    return HAL_OK;
}

/* HMC5883L Heading Calculation */
int16_t HMC5883L_CalculateHeading(HMC5883L_HandleTypeDef *hmc5883l) {
    int16_t heading = 0;

    // Avoid division by zero in case the X component is 0
    if (hmc5883l->x == 0) {
        // Handle special cases for X == 0
        if (hmc5883l->y > 0) {
            heading = 90;  // East
        } else if (hmc5883l->y < 0) {
            heading = 270; // West
        }
    } else {
        // Calculate the angle using the arctangent function (in degrees)
        int32_t angle = (int32_t)(atan2(hmc5883l->y, hmc5883l->x) * 180 / M_PI);

        // Normalize the angle to be in the range of 0-360 degrees
        if (angle < 0) {
            heading = 360 + angle;  // Add 360 to make the angle positive
        } else {
            heading = angle;
        }
    }

    return heading;
}

/* HMC5883L Magnetic Strength Calculation */
void HMC5883L_CalculateMagneticStrength(HMC5883L_HandleTypeDef *hmc5883l) {
    // Calculate the magnetic strength (in microteslas)
    // Using sqrt() to compute the magnitude of the 3D vector formed by x, y, and z components
    hmc5883l->mag_strength = (int16_t)sqrt(hmc5883l->x * hmc5883l->x +
                                            hmc5883l->y * hmc5883l->y +
                                            hmc5883l->z * hmc5883l->z);
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
  MX_ADC1_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  lcd_init();
  lcd_backlight(1);
  lcd_write_string("Initializing...");

  if (BME280_Init(&bme280, &hi2c1, BME280_ADDR) != HAL_OK) {
          lcd_clear();
          lcd_write_string("BME280 Error!");
          while (1);
      }

  if (BH1750_Init(&bh1750, &hi2c1, BH1750_ADDR) != HAL_OK) {
              lcd_clear();
              lcd_set_cursor(1, 0);
              lcd_write_string("BH1750 Init Err");
              while (1); // Infinite loop if sensor initialization fails
          }

  HMC5883L_HandleTypeDef hmc5883l;
  if (HMC5883L_Init(&hmc5883l, &hi2c1, HMC5883L_ADDR) != HAL_OK) {
              lcd_clear();
              lcd_set_cursor(1, 0);
              lcd_write_string("HMC5883L Init Err");
              while (1);
          }

  lcd_clear();
  lcd_write_string("BME280 Ready!");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  int temp, hum, pres;
	            // Declare a buffer to hold the string
	          uint16_t lux;

	          if (BME280_ReadData(&bme280, &temp, &hum, &pres) == HAL_OK) {
	                      char line1[16], line2[16];

	                      snprintf(line1, sizeof(line1), "Temp:%dC Hum:%d%%", temp/100, hum);
	                      snprintf(line2, sizeof(line2), "Press:%d", pres); /*hPa*/

	                      lcd_clear();
	                      lcd_set_cursor(0, 0);
	                      lcd_write_string(line1);
	                      lcd_set_cursor(1, 0);
	                      lcd_write_string(line2);

	                      HAL_Delay(2000);
	                  } else {
	                      lcd_clear();
	                      lcd_set_cursor(0, 0);
	                      lcd_write_string("Read Error!");
	                      HAL_Delay(2000);
	                  }

	                  if (BH1750_ReadLightIntensity(&bh1750, &lux) == HAL_OK) {
	                              // Clear the LCD and display the light intensity
	                              lcd_clear();
	                              lcd_set_cursor(0, 0);


	                              // Display the value (lux) on the second line
	                              char buffer[16];
	                              snprintf(buffer, sizeof(buffer), "Lux:%u ", lux); // Convert the lux value to string
	                              lcd_set_cursor(0, 0); // Move cursor to the second line
	                              lcd_write_string(buffer);
	                              HAL_Delay(2000);
	                          } else {
	                              // Handle error if reading from BH1750 fails
	                              lcd_clear();
	                              lcd_set_cursor(0, 0);
	                              lcd_write_string("Light Read Err");
	                              HAL_Delay(2000);
	                          }





	                  // Initialize HMC5883L
	                  HMC5883L_HandleTypeDef hmc5883l;
	                  if (HMC5883L_Init(&hmc5883l, &hi2c1, HMC5883L_ADDR) != HAL_OK) {
	                      // Error handling
	                  }

	                  // Read data from HMC5883L
	                  if (HMC5883L_ReadData(&hmc5883l) != HAL_OK) {
	                      // Error handling
	                  }

	                  // Calculate Heading and Magnetic Strength
	                  int16_t heading = HMC5883L_CalculateHeading(&hmc5883l);
	                  HMC5883L_CalculateMagneticStrength(&hmc5883l);

	                  // Display heading and strength on LCD (example)
	                  lcd_clear();
	                  lcd_set_cursor(0, 0);
	                  char buffer[32];
	                  sprintf(buffer, "Heading:%d", heading);
	                  lcd_write_string(buffer);

	                  lcd_set_cursor(1, 0);
	                  sprintf(buffer, "Mag Str:%d uT", hmc5883l.mag_strength);
	                  lcd_write_string(buffer);
	                  HAL_Delay(2000);

	                  // Start ADC Conversion
	                  lcd_clear();
	                  lcd_set_cursor(0, 0);
	                  HAL_ADC_Start(&hadc1);

	                  // Wait for ADC conversion to complete
	                  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);

	                  // Read ADC value
	                  uint16_t analog_measurement = HAL_ADC_GetValue(&hadc1);
	                  float analog_volt = (analog_measurement / 4095.0) * 3.3;
	                  int ppm = (116.602 * (pow((10 * (3.3 - analog_volt)) / (9.83 * analog_volt), (-2.769)))) * 100000;/*Might have to calibrated between 1Meg and 10K*/

	                  // Determine air quality
	                  char air_quality[20];
	                  if (ppm <= 400) {
	                      strcpy(air_quality, "Good");
	                  } else if (ppm <= 1000) {
	                      strcpy(air_quality, "Okay");
	                  } else if (ppm <= 1400) {
	                      strcpy(air_quality, "Poor");
	                  } else if (ppm <= 2000) {
	                      strcpy(air_quality, "Bad");
	                  } else {
	                      strcpy(air_quality, "Deadly");
	                  }

	                  // Display PPM and air quality
	                  lcd_set_cursor(0, 0);
	                  sprintf(buffer, "PPM:%d", ppm);
	                  lcd_write_string(buffer);

	                  lcd_set_cursor(1, 0);
	                  sprintf(buffer, "Air Qlt:%s", air_quality);
	                  lcd_write_string(buffer);

	                  // Delay 2 seconds
	                  HAL_Delay(2000);

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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10D19CE4;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
