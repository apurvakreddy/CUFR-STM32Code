/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdint.h>
#include <string.h> //for myprintf
#include <stdarg.h> //for myprintf
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
struct can_msg {
    uint16_t id; //2 bytes
    unsigned char pkt; //1 byte
    uint8_t start; //start byte in the packet, 1 byte
    uint8_t len; //number of bytes of data, 1 byte
    unsigned char bits[3]; //3 bytes (1 bit for whether each bytes enabled--max payload = 22 bytes --> 3 needed)
    uint8_t lts; //number of bytes to send, 1 byte
    uint8_t idx;
};

struct msg_type {
    unsigned char id; //1 byte
    uint8_t data_len; //in bytes, 1 byte
    int speed; //speed to send in ms
    uint8_t num_messages;
    int len;
};

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
HAL_StatusTypeDef ok_notok = HAL_BUSY;   // Tracks HAL SPI call success/failure during SD init

// -------- Raw SD command frames (6 bytes each) --------
// Format: [ command byte | 4-byte argument | CRC ]
// These are sent directly over SPI during low-level SD-card initialization.

uint8_t CMD0 [] = {0x40,0x00,0x00, 0x00, 0x00, 0x95};      // CMD0: software reset, puts SD card into idle/SPI init state
uint8_t high [] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // Dummy clocks sent with MOSI held high before init
uint8_t CMD1 [] = {0x41,0x00,0x00, 0x00, 0x00, 0x79};      // CMD1: legacy init command, kept here for testing/older-card bring-up
uint8_t CMD8 [] = {0x48, 0x00, 0x00, 0x01, 0xAA, 0x87};    // CMD8: checks voltage range / confirms SD v2-style interface
uint8_t CMD55 [] = {0x77, 0x00, 0x00, 0x00, 0x00, 0x65};   // CMD55: prefix indicating next command is application-specific
uint8_t ACMD41 [] = {0x69, 0x40, 0x00, 0x00, 0x00, 0x77};  // ACMD41: application init command, used repeatedly until card exits idle
uint8_t highByte [] = {0xFF};                              // Single dummy byte used between commands / while polling response

uint8_t CMD0_Response [2];      // Stores R1-style response bytes for CMD0
uint8_t CMD1_Response [2];      // Stores response bytes for CMD1
uint8_t CMD8_Response [7];      // Stores CMD8 response + returned check pattern
uint8_t CMD55_Response [2];     // Stores response bytes for CMD55
uint8_t ACMD41_Response [2];    // Stores response bytes for ACMD41

int count = 0;                  // Generic retry counter for init loops / timeout protection

CAN_RxHeaderTypeDef   RxHeader;
uint8_t               RxData[8];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */
void myprintf(const char *fmt, ...);
int process_line(char* msg, struct can_msg* msg_table, int counter);
void get_msg_type_details(char* buffer);
int acdm55(void);
void configure_sd(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void myprintf(const char *fmt, ...) {
  static char buffer[256];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);

  int len = strlen(buffer);
  HAL_UART_Transmit(&huart2, (uint8_t*)buffer, len, -1);

}

// Send the CMD55 + ACMD41 sequence used to bring an SD card out of idle state.
// Returns 1 on success, 0 on failure / timeout.
int acdm55(void) {
	// Send a couple of dummy bytes first to keep SPI clocks running
	// and give the card time between commands.
	ok_notok = HAL_SPI_Transmit(&hspi2, highByte, 1, 1000);
	ok_notok = HAL_SPI_Transmit(&hspi2, highByte, 1, 1000);

	// Send CMD55: tells the card the next command is application-specific.
	ok_notok = HAL_SPI_Transmit(&hspi2, CMD55, 6, 1000); // Send full 6-byte SD command frame
	HAL_Delay(10);                                       // Small delay before reading response
	HAL_SPI_Receive(&hspi2, CMD55_Response, 2, 1000);    // Read response bytes back from card

	// Retry CMD55 until we get the expected response or hit a timeout.
	count = 0;
	while(count < 20 && CMD55_Response[0] != 0x0 && CMD55_Response[1] != 0x1){
		count++;

		// Print unexpected response bytes for debug visibility.
		for(int i = 0; i<2; i++){
			myprintf("(%x)", CMD55_Response[i]);
		}
		myprintf("\r\n");

		// Try CMD55 again.
		ok_notok = HAL_SPI_Transmit(&hspi2, CMD55, 6, 1000);
		HAL_Delay(10);
		HAL_SPI_Receive(&hspi2, CMD55_Response, 2, 1000);
	}

	// If the card never responds correctly, stop here and report failure.
	if(count == 20){
		//myprintf("CMD 55 Timeout \r\n");
		return 0;
	}

	// Print final CMD55 response once it succeeds.
	myprintf("CMD55: ");
	for(int i = 0; i<2; i++){
		myprintf("(%x)", CMD55_Response[i]);
	}
	myprintf("\r\n");

	// Send extra dummy clocks before the next command.
	ok_notok = HAL_SPI_Transmit(&hspi2, highByte, 1, 1000);
	ok_notok = HAL_SPI_Transmit(&hspi2, highByte, 1, 1000);

	// Send ACMD41: actual SD initialization command after CMD55.
	// This is what asks the card to leave idle state and become ready.
	ok_notok = HAL_SPI_Transmit(&hspi2, ACMD41, 6, 1000);
	HAL_Delay(100);                                          // Give card more time for init step
	HAL_SPI_Receive(&hspi2, ACMD41_Response, 2, 1000);       // Read response bytes

	count = 0;

	// If ACMD41 does not return success, print the response and report failure.
	if(ACMD41_Response[0] != 0x0 && ACMD41_Response[1] != 0x0)
	{
		myprintf("ACMD41 Fail once: ");
		for(int i = 0; i<2; i++){
			myprintf("(%x)", ACMD41_Response[i]);
		}
		myprintf("\r\n");
		return 0;
	}

	// Print successful ACMD41 response for debug.
	myprintf("ACMD41: ");
	for(int i = 0; i<2; i++){
		myprintf("(%x)", ACMD41_Response[i]);
	}
	myprintf("\r\n");

	// Both CMD55 and ACMD41 worked, so report success.
	return 1;
}

// ================================================================
// MANUAL SD-CARD SPI STARTUP SEQUENCE
// explicit chip-select control, raw SD init commands, response checking + retry loops, visible success/failure behavior on the board LED.
// ================================================================
void configure_sd(void) {

	// Deassert chip select first so the card is not selected.
	HAL_GPIO_WritePin(GPIOB, SD_CS_Pin, SET);

	// Send 80 dummy clock cycles with MOSI held high.
	// SD cards require startup clocks before entering SPI init mode.
	ok_notok = HAL_SPI_Transmit(&hspi2, high, 10, 1000); // 10 bytes = 80 clocks

	// Assert chip select so following commands are intended for the SD card.
	HAL_GPIO_WritePin(GPIOB, SD_CS_Pin, RESET);

	// ---------------- CMD0 ----------------
	// CMD0 resets the card and requests entry into idle / SPI startup state.
	ok_notok = HAL_SPI_Transmit(&hspi2, CMD0, 6, 1000); // Send 6-byte SD command frame
	HAL_Delay(100);                                     // Give card time to respond
	HAL_SPI_Receive(&hspi2, CMD0_Response, 2, 1000);    // Read response bytes

	// Retry CMD0 until the card reports idle state (0x01) or timeout.
	count = 0;
	while(CMD0_Response[1] != 0x1 && count < 20){
	  count++;
	  myprintf("0 Failed once: \r\n");

	  // Retry the reset command.
	  ok_notok = HAL_SPI_Transmit(&hspi2, CMD0, 6, 1000);
	  HAL_Delay(100);
	  HAL_SPI_Receive(&hspi2, CMD0_Response, 2, 1000);
	}

	// If the card never enters idle state, abort initialization.
	if(count == 20){
		myprintf("Timeout \r\n");
		return;
	}

	// Print final CMD0 response for debug visibility.
	myprintf("CMD0: ");
	for(int i = 0; i<2; i++){
		myprintf("(%x)", CMD0_Response[i]);
	}
	myprintf("\r\n");

	// Send dummy bytes between commands to keep SPI clocks running.
	ok_notok = HAL_SPI_Transmit(&hspi2, highByte, 1, 1000);
	ok_notok = HAL_SPI_Transmit(&hspi2, highByte, 1, 1000);

	// ---------------- CMD8 ----------------
	// CMD8 checks interface conditions:
	// - confirms voltage support
	// - helps distinguish newer SD cards from older behavior
	ok_notok = HAL_SPI_Transmit(&hspi2, CMD8, 6, 1000);
	HAL_Delay(100);
	HAL_SPI_Receive(&hspi2, CMD8_Response, 7, 1000);

	// Retry until response shows idle state and echoed check pattern 0xAA.
	count = 0;
	while(count < 20 && (CMD8_Response[1] != 0x1 || CMD8_Response[5] != 0xAA)){
		count++;
		myprintf("8 Failed once: \r\n");

		// Print returned bytes so we can see exactly how the card responded.
		for(int i = 0; i<7; i++){
			myprintf("(%x)", CMD8_Response[i]);
		}

		// Retry CMD8.
		ok_notok = HAL_SPI_Transmit(&hspi2, CMD8, 6, 1000);
		HAL_Delay(100);
		HAL_SPI_Receive(&hspi2, CMD8_Response, 7, 1000);
	}

	// Abort if CMD8 never gives the expected response.
	if(count == 20){
		myprintf("Timeout \r\n");
		return;
	}

	// Print final CMD8 response for debug.
	myprintf("CMD8: ");
	for(int i = 0; i<7; i++){
		myprintf("(%x)", CMD8_Response[i]);
	}
	myprintf("\r\n");

	// Send dummy bytes before next command group.
	ok_notok = HAL_SPI_Transmit(&hspi2, highByte, 1, 1000);
	ok_notok = HAL_SPI_Transmit(&hspi2, highByte, 1, 1000);

	// ---------------- CMD55 + ACMD41 ----------------
	// CMD55 tells the card the next command is application-specific.
	// ACMD41 is the actual init command that asks the card to leave idle state.
	int counter = 0;
	int worked = acdm55();

	myprintf("Worked: (%i)\r\n", worked);

	// Keep retrying until the card is ready or we hit a timeout.
	while(worked == 0 && counter < 20){
		worked = acdm55();
		counter++;

		if(worked == 1){
			myprintf("yay!\r\n"); // Card finally accepted init sequence
		}
	}

	myprintf("Count: %i\r\n", counter);
	myprintf("Worked: %i\r\n", worked);

	// Abort if card never exits idle state.
	if(counter == 20){
		myprintf("Timeout\r\n");
		return;
	}

	myprintf("End of Start Up\r\n");

	// If the last HAL transaction succeeded, turn on LED as visible success signal.
	if(ok_notok == HAL_OK){
	  HAL_GPIO_WritePin(GPIOA, Green_LED_Pin, SET);
	}

	// Hold LED on briefly so successful init is obvious on the board.
	HAL_Delay(1000);

	// Turn LED back off after startup indication.
	HAL_GPIO_WritePin(GPIOA, Green_LED_Pin, RESET);

	// Deassert chip select when done so the SD card is no longer selected.
	HAL_GPIO_WritePin(GPIOB, SD_CS_Pin, SET);
}


struct msg_type med;
struct msg_type fast;
struct msg_type stat;

int process_line(char* msg, struct can_msg* msg_table, int counter) {
    struct can_msg M = { 0 };
    M.id = 0;
    M.id |= msg[0] << 8;
    M.id |= msg[1];
    M.pkt = msg[2];
    M.start = msg[3];
    M.len = msg[4];
    M.bits[0] = msg[5];
    M.bits[1] = msg[6];
    M.bits[2] = msg[7];
    M.lts = msg[8];
    M.idx = msg[9];
    msg_table[counter] = M;
    return counter++;
}

void get_msg_type_details(char* buffer) {
    uint8_t s;

    //Get the details for the medium speed message
    med.id = buffer[1];
    med.data_len = buffer[2];
    s = buffer[3];
    med.speed = 10 * (int)s;
    med.num_messages = buffer[4];
    med.len = med.data_len + 3;

    myprintf("Med\r\n");
    myprintf("id: %c\r\n", med.id);
    myprintf("data_len: %u\r\n", med.data_len);
    myprintf("speed: %i\r\n", med.speed);
    myprintf("num_messages: %u\r\n", med.num_messages);

    //Get the details for the fast speed message
    fast.id = buffer[5];
    fast.data_len = buffer[6];
    s = buffer[7];
    fast.speed = 10 * (int)s;
    fast.num_messages = buffer[8];
    fast.len = fast.data_len + 3;

    myprintf("Fast\r\n");
    myprintf("id: %c\r\n", fast.id);
    myprintf("data_len: %u\r\n", fast.data_len);
    myprintf("speed: %i\r\n", fast.speed);
    myprintf("num_messages: %u\r\n", fast.num_messages);

    //Get the details for the stat speed message
    stat.id = buffer[9];
    stat.data_len = buffer[10];
    s = buffer[11];
    stat.speed = 10 * (int)s;
    stat.num_messages = buffer[12];
    stat.len = stat.data_len + 3;

    myprintf("Stat\r\n");
    myprintf("id: %c\r\n", stat.id);
    myprintf("data_len: %u\r\n", stat.data_len);
    myprintf("speed: %i\r\n", stat.speed);
    myprintf("num_messages: %u\r\n", stat.num_messages);
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
  MX_CAN_Init();
  MX_USART1_UART_Init();
  MX_SPI2_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */
  myprintf("\r\n~ Starting LT~\r\n\r\n");
  configure_sd();
  HAL_Delay(1000); //a short delay is important to let the SD card settle
  //some variables for FatFs
  FATFS FatFs; 	//Fatfs handle
  FIL fil; 		//File handle
  FRESULT fres; //Result after operations

  //Set Up CAN
  CAN_FilterTypeDef canfilterconfig;
  canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
  canfilterconfig.FilterBank = 8;  // which filter bank to use from the assigned ones
  canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  canfilterconfig.FilterIdHigh = 0x446<<5;
  canfilterconfig.FilterIdLow = 0;
  canfilterconfig.FilterMaskIdHigh = 0x0000;
  canfilterconfig.FilterMaskIdLow = 0x0000;
  canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
  canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
  canfilterconfig.SlaveStartFilterBank = 14;  // how many filters to assign to the CAN1 (master can)

  HAL_CAN_ConfigFilter(&hcan, &canfilterconfig);
  HAL_StatusTypeDef ret = HAL_CAN_Start(&hcan);
  if(ret != HAL_OK){
  	  myprintf("\rinit error\r\n\r\n");
    }
    else
    {
  	  myprintf("\rinitialised\r\n\r\n");
    }

  //Open the file system
  myprintf("\r\nInitialising SD CARD\r\n\r\n");
  int init_tries = 0;
  fres = f_mount(&FatFs, "", 1); //1=mount now
  while(fres != FR_OK && init_tries<5){
	myprintf("Trying Again\r\n");
	init_tries++;
	fres = f_mount(&FatFs, "", 1); //1=mount now
  }
  if (fres != FR_OK) {
	myprintf("f_mount error (%i)\r\n", fres);
	while(1){
		HAL_GPIO_TogglePin(GPIOA, Green_LED_Pin);
		HAL_Delay(500);
		HAL_GPIO_TogglePin(GPIOA, Green_LED_Pin);
		HAL_Delay(100);
	}
  }
  myprintf("\r\nSD CARD Initialised!\r\n\r\n");

  char buffer[255];
  char *filename = "settings.bin";

  fres = f_open(&fil, filename, FA_READ);
  if (fres != FR_OK) {
	myprintf("f_open error (%i)\r\n", fres);
	myprintf("unable to open file");
  }
  else{
	myprintf("Settings File opened!\r\n");
  }

  //get number of messages
  UINT bytes_read;
  fres = f_read(&fil, buffer, 13, &bytes_read);
  if(bytes_read == 0) {
	myprintf("f_read error (%i)\r\n", fres);
  }
  uint8_t num_msgs = buffer[0];
  myprintf("num messages: %u\r\n", num_msgs);

  struct can_msg table[num_msgs];
  //Get message types
  get_msg_type_details(buffer);

  //get individual message details
  myprintf("Getting individual msg details\r\n");
  for (int counter = 0; counter < num_msgs; counter++) {
      f_read(&fil, buffer, 10, &bytes_read);
      process_line(buffer, table, counter);
  }
  f_close(&fil);

  myprintf("\r\nMessage Information Loaded\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  //remove once CAN works
//  filename = "fake_can.bin";
//  fres = f_open(&fil, filename, FA_READ);
//	if (fres != FR_OK) {
//		myprintf("f_open error (%i)\r\n", fres);
//		myprintf("unable to open fake messages file");
//	}
//	else{
//		myprintf("Msg File opened!\r\n");
//	}
//
//    //get number of messages
//    fres = f_read(&fil, buffer, 1, &bytes_read);
//    if(bytes_read == 0) {
//  	myprintf("f_read error (%i)\r\n", fres);
//    }
//    uint8_t num_csv_msgs = buffer[0];
//    myprintf("num messages: %u\r\n", num_csv_msgs);

    //int msg_ctr = 0;
    struct can_msg msg_type;
    struct can_msg null_msg;
    null_msg.id = 0;
    uint32_t fill_level = 0;

    uint32_t stat_last_time = HAL_GetTick();
    uint32_t fast_last_time = HAL_GetTick();
    uint32_t med_last_time = HAL_GetTick();
    uint32_t curr_time;

    uint8_t stat_pkt[stat.data_len+3];
    uint8_t stat_pkt_last[stat.data_len+3];
    stat_pkt[0] = stat.id;
    uint8_t med_pkt[med.data_len+3];
    med_pkt[0] = med.id;
    uint8_t fast_pkt[fast.data_len+3];
    fast_pkt[0] = fast.id;

    uint8_t *curr_pkt;
    uint8_t send_stat = 0;

    stat_pkt[1] = stat_pkt[2] = 0;
    med_pkt[1] = med_pkt[2] = 0;
    fast_pkt[1] = fast_pkt[2] = 0;

  while (1)
  {
	  //	  - check whether there are any CAN messages to be read
	  //	  	  - read a message
	  //	  	  - get the header and search to find the appropriate message (done)
	  //	  	  - add to the relevant packet. Mark bit as being updated
	  //	  	  if(is stat or time since last stat sent > stat time):
	  //	  	  - Send stat
	  //	  	  if(time since last fast sent > fast time):
	  //	  	  - send fast
	  //	  	  -if(time since last med sent > med time):
	  //	  	  - send med
	  //	  	  repeat;

//	  myprintf("Next msg\r\n");
//	  fres = f_read(&fil, buffer, 16, &bytes_read);
//	  if(bytes_read == 0) {
//		myprintf("f_read error (%i)\r\n", fres);
//	  }
//
//	  RxHeader.StdId = 0;
//	  RxHeader.StdId |= buffer[0] << 24;
//	  RxHeader.StdId |= buffer[1] << 16;
//	  RxHeader.StdId |= buffer[2] << 8;
//	  RxHeader.StdId |= buffer[3];
//
//	  RxHeader.DLC = 0;
//	  RxHeader.DLC |= buffer[4] << 24;
//	  RxHeader.DLC |= buffer[5] << 16;
//	  RxHeader.DLC |= buffer[6] << 8;
//	  RxHeader.DLC |= buffer[7];
//
//	  for(int idx = 0; idx<RxHeader.DLC; idx++){
//		 RxData[idx] = buffer[8+idx];
//	  }
//
//	  myprintf("(%x) (%i) ", RxHeader.StdId, RxHeader.DLC);
//	  for(int idx = 0; idx<RxHeader.DLC; idx++){
//		  myprintf("(%x)", RxData[idx]);
//	  	  }
//	  myprintf("\r\n");

	  fill_level = HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0);
	  if(fill_level !=0){
		 if (HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
		 {
				 myprintf("No msg found\r\n\r\n");
		  }else {

		  //Check which message was received
		  	  msg_type = null_msg;
		  	  for(int idx =0; idx<num_msgs; idx++){
		  		  if(RxHeader.StdId == table[idx].id){
		  			  msg_type = table[idx];
		  			  break;
		  		  }
		  	  }

		  	  if(msg_type.id != 0)
		  	  {
		  		  myprintf("(%x) (%c)\r\n", msg_type.id, msg_type.pkt);
		  	  }
		  	  else{
		  		  myprintf("Corresponding message not found\r\n");
		  	  }

		  	  curr_pkt = NULL;

		  	  //Sets the curr_pkt pointer to the correct packet
		  	  if(msg_type.pkt == fast.id)
		  	  {
		  		  curr_pkt = fast_pkt;
		  	  } else if(msg_type.pkt == med.id){
		  		  curr_pkt = med_pkt;
		  	  } else if(msg_type.pkt == stat.id){
		  		  curr_pkt = stat_pkt;
		  	  }

		  	  //Check if the stat_pkt has changed
		  	  if(msg_type.pkt == stat.id){
		  		  for(int idx = 0; idx<msg_type.lts; idx++){
		  			  if(curr_pkt[msg_type.start+idx] != stat_pkt_last[idx])
		  			  {
		  				  send_stat = 1;
		  				  myprintf("Stat changed!!\r\n");
		  			  }
		  		  }
		  	  }

		  	  //Marks the set bit
		  	  if(curr_pkt != NULL && msg_type.idx < 8){
		  		  curr_pkt[1] |= 1U << (7-msg_type.idx);
		  		  //myprintf("1(%u)\r\n", curr_pkt[1]);
		  	  } else if (curr_pkt != NULL){
		  		  curr_pkt[2] |= 1U << (15-msg_type.idx);
		  		  //myprintf("2(%u)\r\n", curr_pkt[2]);
		  	  }

		  	  //Adds the message's data to the packet
		  	  if(curr_pkt != NULL){
		  		  for(int idx = 0; idx<msg_type.lts; idx++){
		  			  curr_pkt[msg_type.start+idx] = RxData[idx];
		  		  }
		  	  }
		  }
	  }



	  //Checks which messages to send
	  curr_time = HAL_GetTick();
	  if(send_stat == 1 || curr_time - stat_last_time > stat.speed*100){
		  send_stat = 0;
		  ok_notok = HAL_UART_Transmit(&huart1, stat_pkt, stat.data_len+3, 100);
		  for(int idx = 0; idx<stat.data_len+3; idx++){
		  			  myprintf("%2x", stat_pkt[idx]);
		  		  }
		  myprintf("\r\n");
		  myprintf("lts: (%i)\r\n", stat.data_len+3);
		  myprintf("Stat sent!!\r\n");
		  stat_pkt[1] = 0;
		  stat_pkt[2] = 0;
		  stat_last_time = curr_time;
	  }

	  if(curr_time - fast_last_time > fast.speed*100){
		  ok_notok = HAL_UART_Transmit(&huart1, fast_pkt, fast.data_len+3, 100);
		  for(int idx = 0; idx<fast.data_len+3; idx++){
			  myprintf("%2x", fast_pkt[idx]);
		  }
		  myprintf("\r\n");
		  myprintf("Fast sent!!\r\n");
		  med_pkt[1] = 0;
		  med_pkt[2] = 0;
		  fast_last_time = curr_time;
	  }

	  if(curr_time - med_last_time > med.speed*100){
		  ok_notok = HAL_UART_Transmit(&huart1, med_pkt, med.data_len+3, 100);
		  med_pkt[1] = 0;
		  med_pkt[2] = 0;
		  med_last_time = curr_time;
	  }

	  //Remove once CAN works
//	  	  msg_ctr++;
//	  	  if(msg_ctr == num_csv_msgs)
//	  	  {
//	  		msg_ctr = 0;
//	  		fres = f_lseek(&fil, 1);
//	  	  }
//	  	  HAL_Delay(1000);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN;
  hcan.Init.Prescaler = 9;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  huart1.Init.HwFlowCtl = UART_HWCONTROL_RTS_CTS;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Green_LED_GPIO_Port, Green_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Green_LED_Pin */
  GPIO_InitStruct.Pin = Green_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Green_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_CS_Pin */
  GPIO_InitStruct.Pin = SD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SD_CS_GPIO_Port, &GPIO_InitStruct);

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
