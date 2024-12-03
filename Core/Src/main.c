/* USER CODE BEGIN Header */
/**
  ****************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ****************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ****************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32l4s5i_iot01.h"
#include "stm32l4s5i_iot01_gyro.h"
#include "stm32l4s5i_iot01_accelero.h"
#include "../../Drivers/Components/hts221/hts221.h"
#include "../../Drivers/Components/lsm6dsl/lsm6dsl.h"
#include "stm32l4s5i_iot01_qspi.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define PRECISION 4096
#define NOTE_CAPACITY 42
#define HIGHSCORE_FLASH_ADDRESS 0x00000000
#define SEED_FLASH_ADDRESS 0x00000400
#define DEFAULT_SEED 40




/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac1_ch1;

I2C_HandleTypeDef hi2c1;

OSPI_HandleTypeDef hospi1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

uint32_t counter = 0;
int gameover = 0;
uint32_t highScore = 0;
uint32_t array_size[] = {42,38,34,28};
uint32_t C6[NOTE_CAPACITY];
uint32_t D6[NOTE_CAPACITY];
uint32_t E6[NOTE_CAPACITY];
uint32_t G6[NOTE_CAPACITY];

uint32_t *notes[4];



uint32_t victoryMelody[] = {
    0, 2, 3, 0, 3, 0, 0   // C6, E6, G6, C7, G6, C7, C7
};

uint32_t victoryDurations[] = {
    500, 500, 500, 500, 300, 300, 1000 // Durations in milliseconds for each note
};


uint32_t victoryLength = 7; // Number of notes in the victory melody
uint32_t victoryIndex = 0;  // Index to track the current note
uint32_t dac_array[16];


int treasureRow, treasureCol;
int playerRow = 0;
int playerCol = 0;

// thresholds for movement
const int16_t accelThreshold = 200;
const float gyroThreshold = 1000.0f;

// movement tracking

// use current as reference
int currentDirection = 0;

// debounce to help with noise
uint32_t lastChangeTime = 0;
const uint32_t debounceTime = 500; // Minimum time (ms) between direction changes

int sizeRow;
int sizeCol;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_DAC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_OCTOSPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */



void SaveHighScore(uint32_t highScore) {
    uint8_t dataToWrite[4];

    // Set the highest bit to indicate that the high score is valid
    highScore |= (1UL << 31);

    // Convert the high score into a 4-byte array
    dataToWrite[0] = (highScore >> 24) & 0xFF;
    dataToWrite[1] = (highScore >> 16) & 0xFF;
    dataToWrite[2] = (highScore >> 8) & 0xFF;
    dataToWrite[3] = highScore & 0xFF;

    if (BSP_QSPI_Init() != QSPI_OK) {
//        HAL_UART_Transmit(&huart1, (uint8_t *)"QSPI Init Failed\r\n", 18, HAL_MAX_DELAY);
        Error_Handler();
    }

//    HAL_UART_Transmit(&huart1, (uint8_t *)"QSPI Init Success\r\n", 20, HAL_MAX_DELAY);

    // Erase the flash sector containing the high score
    if (BSP_QSPI_Erase_Sector(HIGHSCORE_FLASH_ADDRESS) != QSPI_OK) {
//        HAL_UART_Transmit(&huart1, (uint8_t *)"QSPI Erase Failed\r\n", 19, HAL_MAX_DELAY);
        Error_Handler();
    }

//    HAL_UART_Transmit(&huart1, (uint8_t *)"QSPI Erase Command Sent\r\n", 25, HAL_MAX_DELAY);

    // Wait for the erase operation to complete
    while (BSP_QSPI_GetStatus() == QSPI_BUSY) {
        HAL_Delay(10);
    }

//    HAL_UART_Transmit(&huart1, (uint8_t *)"QSPI Erase Completed\r\n", 23, HAL_MAX_DELAY);

    // Write the high score to the flash memory
    if (BSP_QSPI_Write(dataToWrite, HIGHSCORE_FLASH_ADDRESS, sizeof(dataToWrite)) != QSPI_OK) {
//        HAL_UART_Transmit(&huart1, (uint8_t *)"QSPI Write Failed\r\n", 20, HAL_MAX_DELAY);
        Error_Handler();
    }

    HAL_UART_Transmit(&huart1, (uint8_t *)"High Score Saved\r\n", 18, HAL_MAX_DELAY);
}


uint32_t LoadHighScore(void) {
    uint8_t dataRead[4]; // Buffer to hold the 4 bytes of high score data

    // Initialize QSPI
    if (BSP_QSPI_Init() != QSPI_OK) {
//        HAL_UART_Transmit(&huart1, (uint8_t *)"QSPI Init Failed\r\n", 18, HAL_MAX_DELAY);
        Error_Handler();
    }

    HAL_UART_Transmit(&huart1, (uint8_t *)"QSPI Init Success\r\n", 20, HAL_MAX_DELAY);

    // Read high score from flash memory
    if (BSP_QSPI_Read(dataRead, HIGHSCORE_FLASH_ADDRESS, sizeof(dataRead)) != QSPI_OK) {
//        HAL_UART_Transmit(&huart1, (uint8_t *)"QSPI Read Failed\r\n", 19, HAL_MAX_DELAY);
        Error_Handler();
    }

    // Combine the 4 bytes into a 32-bit integer
    uint32_t loadedHighScore = (dataRead[0] << 24) | (dataRead[1] << 16) | (dataRead[2] << 8) | dataRead[3];

    // Check if the highest bit (bit 31) is set, indicating a valid high score
    if ((loadedHighScore & (1UL << 31)) == 0) {
        // If bit 31 is not set, assume the high score is uninitialized and set it to 1000
        loadedHighScore = 10000;
    } else {
        // Clear the highest bit before using the high score value
        loadedHighScore &= ~(1UL << 31);
    }

    // Print the high score


    return loadedHighScore; // Return the high score
}

void SaveSeed(uint32_t seed) {
    uint8_t dataToWrite[4];

    dataToWrite[0] = (seed >> 24) & 0xFF;
    dataToWrite[1] = (seed >> 16) & 0xFF;
    dataToWrite[2] = (seed >> 8) & 0xFF;
    dataToWrite[3] = seed & 0xFF;

    if (BSP_QSPI_Init() != QSPI_OK) {
//        HAL_UART_Transmit(&huart1, (uint8_t *)"QSPI Init Failed\r\n", 18, HAL_MAX_DELAY);
        Error_Handler();
    }

    if (BSP_QSPI_Erase_Sector(SEED_FLASH_ADDRESS) != QSPI_OK) {
//        HAL_UART_Transmit(&huart1, (uint8_t *)"QSPI Erase Failed\r\n", 19, HAL_MAX_DELAY);
        Error_Handler();
    }

    while (BSP_QSPI_GetStatus() == QSPI_BUSY) {
        HAL_Delay(10);
    }

    if (BSP_QSPI_Write(dataToWrite, SEED_FLASH_ADDRESS, sizeof(dataToWrite)) != QSPI_OK) {
//        HAL_UART_Transmit(&huart1, (uint8_t *)"QSPI Write Failed\r\n", 20, HAL_MAX_DELAY);
        Error_Handler();
    }
}

uint32_t LoadSeed(void) {
    uint8_t dataRead[4];

    if (BSP_QSPI_Init() != QSPI_OK) {
//        HAL_UART_Transmit(&huart1, (uint8_t *)"QSPI Init Failed\r\n", 18, HAL_MAX_DELAY);
        Error_Handler();
    }

    if (BSP_QSPI_Read(dataRead, SEED_FLASH_ADDRESS, sizeof(dataRead)) != QSPI_OK) {
//        HAL_UART_Transmit(&huart1, (uint8_t *)"QSPI Read Failed\r\n", 19, HAL_MAX_DELAY);
        Error_Handler();
    }

    uint32_t seed = (dataRead[0] << 24) | (dataRead[1] << 16) | (dataRead[2] << 8) | dataRead[3];

    if (seed == 0xFFFFFFFF) {
        seed = DEFAULT_SEED;
    }

    return seed;
}

int DetectMovement(void) {
	// measure
    float gyroData[3];
    int16_t accelData[3];
    BSP_GYRO_GetXYZ(gyroData);
    BSP_ACCELERO_AccGetXYZ(accelData);

    // direction string to print
    int newDirection = 0;

    // current time
    uint32_t currentTime = HAL_GetTick();

    // check tilt
    if (accelData[0] > accelThreshold && gyroData[0] > gyroThreshold) {
    	newDirection = 1; //up
    } else if (accelData[1] < -accelThreshold && gyroData[1] < -gyroThreshold) {
    	newDirection = 2; //left
    } else if (accelData[0] < -accelThreshold && gyroData[0] < -gyroThreshold) {
    	newDirection = 3; //down
    } else if (accelData[1] > accelThreshold && gyroData[1] > gyroThreshold) {
    	newDirection = 4; //right
    } else {
    	newDirection = 0; //neutral
    }

    // only print if change in direction after enough delay (debounce)
    if ((newDirection != currentDirection) && (currentTime - lastChangeTime > debounceTime)) {
        // update direction
    	currentDirection = newDirection;
        lastChangeTime = currentTime;

        // print, for debug since we'll return direction
        if (currentDirection != 0) {
            char buf[50];
            snprintf(buf, sizeof(buf), "Move %d\r\n", currentDirection);
            HAL_UART_Transmit(&huart1, (uint8_t *)buf, strlen(buf), HAL_MAX_DELAY);
        }
    }

    return currentDirection;
}

void PrintInitialGrid(void) {
    const char newline[] = "\r\n"; // Newline for the terminal
    char displayBuffer[64];

    // Print a grid of all '1's
    for (int i = 0; i < 4; i++) {
        memset(displayBuffer, '1', 4);
        displayBuffer[4] = '\0';
        HAL_UART_Transmit(&huart1, (uint8_t *)displayBuffer, strlen(displayBuffer), HAL_MAX_DELAY);
        HAL_UART_Transmit(&huart1, (uint8_t *)newline, strlen(newline), HAL_MAX_DELAY);
    }
}

void PrintTreasureGrid(void) {
    const char newline[] = "\r\n";
    char displayBuffer[128];
    char grid[sizeRow][sizeCol];

    for (int i = 0; i < sizeRow; i++) {
        for (int j = 0; j < sizeCol; j++) {
            grid[i][j] = '.';
        }
    }

    grid[playerRow][playerCol] = '*';
    grid[treasureRow][treasureCol] = 'x';


    // Print the top border
    snprintf(displayBuffer, sizeof(displayBuffer), "+");
    for (int j = 0; j < sizeCol; j++) {
        strncat(displayBuffer, "---+", sizeof(displayBuffer) - strlen(displayBuffer) - 1);
    }
    strncat(displayBuffer, newline, sizeof(displayBuffer) - strlen(displayBuffer) - 1);
    HAL_UART_Transmit(&huart1, (uint8_t *)displayBuffer, strlen(displayBuffer), HAL_MAX_DELAY);

    for (int i = 0; i < sizeRow; i++) {
        char rowBuffer[32] = "|";
        for (int j = 0; j < sizeCol; j++) {
            char cell[8];
            snprintf(cell, sizeof(cell), " %c |", grid[i][j]);
            strncat(rowBuffer, cell, sizeof(rowBuffer) - strlen(rowBuffer) - 1);
        }
        snprintf(displayBuffer, sizeof(displayBuffer), "%s\r\n", rowBuffer);
        HAL_UART_Transmit(&huart1, (uint8_t *)displayBuffer, strlen(displayBuffer), HAL_MAX_DELAY);

        snprintf(displayBuffer, sizeof(displayBuffer), "+");
        for (int j = 0; j < sizeCol; j++) {
            strncat(displayBuffer, "---+", sizeof(displayBuffer) - strlen(displayBuffer) - 1);
        }
        strncat(displayBuffer, newline, sizeof(displayBuffer) - strlen(displayBuffer) - 1);
        HAL_UART_Transmit(&huart1, (uint8_t *)displayBuffer, strlen(displayBuffer), HAL_MAX_DELAY);
    }
}



void Move(void) {
    int movement = 0;

    // wait to get a valid direction (not none)
    while (movement == 0) {
        movement = DetectMovement();
        HAL_Delay(50);
    }

    // move
    switch (movement) {
        case 2: // up
            if (playerRow > 0) {
                playerRow--;
                counter++;
            }
            break;
        case 3: // right
            if (playerCol < sizeCol) {
                playerCol++;
                counter++;
            }
            break;
        case 4: // down
            if (playerRow < sizeRow) {
                playerRow++;
                counter++;
            }
            break;
        case 1: // left
            if (playerCol > 0) {
                playerCol--;
                counter++;
            }
            break;
        default:
            break;
    }

    // show new map
    PrintTreasureGrid();

    // print the updated number of moves
    char counterBuffer[50];
    snprintf(counterBuffer, sizeof(counterBuffer), "Moves made: %d\r\n", counter);
    HAL_UART_Transmit(&huart1, (uint8_t *)counterBuffer, strlen(counterBuffer), HAL_MAX_DELAY);

    // check if on treasure
    if (playerRow == treasureRow && playerCol == treasureCol) {
        const char winMessage[] = "You found the treasure!\r\n";
        HAL_UART_Transmit(&huart1, (uint8_t *)winMessage, strlen(winMessage), HAL_MAX_DELAY);
        HAL_Delay(1000);

        if(counter < LoadHighScore()){

        SaveHighScore(counter);

        }







        for(int i = 0; i< victoryLength; i++){


         uint32_t nextNote = victoryMelody[i]; // Get the current note
   	     uint32_t duration = victoryDurations[i]; // Get the current duration

   	     // Stop the current DMA for buffer
   	     HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);

   	     // Start DMA with the new note
   	     HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, notes[nextNote], array_size[nextNote], DAC_ALIGN_12B_R);

   	     // Add a delay for the note duration
   	     HAL_Delay(duration);
        }

        HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
        gameover = 1;

   	     // Increment the index to play the next note

        // maybe reset here?
    }
    else {
    	if (playerRow > treasureRow && playerCol > treasureCol) {
    		const char winMessage[] = "The treasure further up, maybe left?\r\n";
    		HAL_UART_Transmit(&huart1, (uint8_t *)winMessage, strlen(winMessage), HAL_MAX_DELAY);
    		HAL_Delay(1000);
    	}
    	else if(playerRow > treasureRow && playerCol < treasureCol) {
    		const char winMessage[] = "You should go right up:))\r\n";
    		HAL_UART_Transmit(&huart1, (uint8_t *)winMessage, strlen(winMessage), HAL_MAX_DELAY);
    		HAL_Delay(1000);
    	}
    	else if(playerRow < treasureRow && playerCol > treasureCol) {
    		const char winMessage[] = "going up is not ~right~\r\n";
    		HAL_UART_Transmit(&huart1, (uint8_t *)winMessage, strlen(winMessage), HAL_MAX_DELAY);
    		HAL_Delay(1000);
    	}
    	else if(playerRow < treasureRow && playerCol < treasureCol) {
    		const char winMessage[] = "further down, right?\r\n";
    		HAL_UART_Transmit(&huart1, (uint8_t *)winMessage, strlen(winMessage), HAL_MAX_DELAY);
    		HAL_Delay(1000);
    	}
    	else if(playerRow == treasureRow) {
    		const char winMessage[] = "you're at the right level!\r\n";
    		HAL_UART_Transmit(&huart1, (uint8_t *)winMessage, strlen(winMessage), HAL_MAX_DELAY);
    		HAL_Delay(1000);
    	}
    	else if(playerCol == treasureCol) {
    		const char winMessage[] = "you should keep climbing up, or maybe down? you decide.\r\n";
    		HAL_UART_Transmit(&huart1, (uint8_t *)winMessage, strlen(winMessage), HAL_MAX_DELAY);
    		HAL_Delay(1000);
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

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  uint32_t seed = LoadSeed();
  srand(seed);
  uint32_t nextSeed = rand();
  SaveSeed(nextSeed);

  sizeRow = rand()%6 + 4;
  sizeCol = rand()%6 + 4;

  // Print sizeRow and sizeCol
  char debugBuffer[50];
  snprintf(debugBuffer, sizeof(debugBuffer), "Col: %d, Row: %d\r\n", sizeCol, sizeRow);
  HAL_UART_Transmit(&huart1, (uint8_t *)debugBuffer, strlen(debugBuffer), HAL_MAX_DELAY);
  treasureRow = rand() % sizeRow; // Random row (0-3)
  treasureCol = rand() % sizeCol; // Random column (0-3)
//  uint32_t resetHS = 1000;
//  SaveHighScore(resetHS);
  BSP_GYRO_Init();
  BSP_ACCELERO_Init();

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_DAC1_Init();
  MX_I2C1_Init();
  MX_OCTOSPI1_Init();
  /* USER CODE BEGIN 2 */

  notes[0] = C6;
   notes[1] = D6;
   notes[2] = E6;
   notes[3] = G6;

 for (int i = 0; i<4; i++) {
  generateArray(notes[i], PRECISION, array_size[i]);
 }


 char highScoreBuffer[50];
 snprintf(highScoreBuffer, sizeof(highScoreBuffer), "High Score: %d\r\n", LoadHighScore());
 HAL_UART_Transmit(&huart1, (uint8_t *)highScoreBuffer, strlen(highScoreBuffer), HAL_MAX_DELAY);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

//  PrintInitialGrid();
  // Print the updated number of moves

  const char newline[] = "\r\n";
  HAL_UART_Transmit(&huart1, (uint8_t *)newline, strlen(newline), HAL_MAX_DELAY);
  PrintTreasureGrid();
  while (!gameover)
  {
	  Move();
	  HAL_Delay(1500);


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_T2_TRGO;
  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_ABOVE_80MHZ;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT2 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

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
  hi2c1.Init.Timing = 0x30A175AB;
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
  * @brief OCTOSPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_OCTOSPI1_Init(void)
{

  /* USER CODE BEGIN OCTOSPI1_Init 0 */

  /* USER CODE END OCTOSPI1_Init 0 */

  OSPIM_CfgTypeDef OSPIM_Cfg_Struct = {0};

  /* USER CODE BEGIN OCTOSPI1_Init 1 */

  /* USER CODE END OCTOSPI1_Init 1 */
  /* OCTOSPI1 parameter configuration*/
  hospi1.Instance = OCTOSPI1;
  hospi1.Init.FifoThreshold = 1;
  hospi1.Init.DualQuad = HAL_OSPI_DUALQUAD_DISABLE;
  hospi1.Init.MemoryType = HAL_OSPI_MEMTYPE_MICRON;
  hospi1.Init.DeviceSize = 23;
  hospi1.Init.ChipSelectHighTime = 1;
  hospi1.Init.FreeRunningClock = HAL_OSPI_FREERUNCLK_DISABLE;
  hospi1.Init.ClockMode = HAL_OSPI_CLOCK_MODE_0;
  hospi1.Init.ClockPrescaler = 1;
  hospi1.Init.SampleShifting = HAL_OSPI_SAMPLE_SHIFTING_HALFCYCLE;
  hospi1.Init.DelayHoldQuarterCycle = HAL_OSPI_DHQC_DISABLE;
  hospi1.Init.ChipSelectBoundary = 0;
  hospi1.Init.DelayBlockBypass = HAL_OSPI_DELAY_BLOCK_BYPASSED;
  if (HAL_OSPI_Init(&hospi1) != HAL_OK)
  {
    Error_Handler();
  }
  OSPIM_Cfg_Struct.ClkPort = 1;
  OSPIM_Cfg_Struct.NCSPort = 1;
  OSPIM_Cfg_Struct.IOLowPort = HAL_OSPIM_IOPORT_1_LOW;
  if (HAL_OSPIM_Config(&hospi1, &OSPIM_Cfg_Struct, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN OCTOSPI1_Init 2 */

  /* USER CODE END OCTOSPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 2721;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start(&htim2);

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
