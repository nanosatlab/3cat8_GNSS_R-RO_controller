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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "types.h"
#include "string.h"
#include "nmea.h"
#include "configure_receiver_commands.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
FATFS fs;
FATFS *pfs;
FIL filB16, filB17;
UINT BW = 0;
FRESULT fres;
char pvt_reserve[200];

#define SD_card_enabled 				0
#define B16_ENABLED						0
#define B17_ENABLED						1
#define UPDATE_TO_SRAM					0x00
#define UPDATE_TO_SRAM_AND_FLASH		0x01

#define BUFFER_SIZE_B16					100 //150
#define BUFFER_SIZE_B17					700
#define UART_B16						huart4
#define UART_B17						huart1
#define UART_OBC						huart2


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */
volatile int buffer_full_B16 = 0;
volatile int buffer_full_B17 = 0;
volatile int callback_index = 0;

uint8_t buffer_B16[2*BUFFER_SIZE_B16];
uint8_t buffer_B17[2*BUFFER_SIZE_B17];

int cmd_sent_B16 = 0;
int cmd_sent_B17 = 0;
int ack_flag_B16 = 0, nack_flag_B16 = 0;
int ack_flag_B17 = 0, nack_flag_B17 = 0;
int navigation_message_counter = 0;
char* num_to_constellation[7] = {"GPS", "SBA", "GLO", "GAL", "QZS", "BEI", "IRN"};
uint8_t enter_doze_mode[9] = {0xA0, 0xA1, 0x00, 0x02, 0x64, 0x1C, 0x78, 0x0D, 0x0A};
uint8_t set_factory_defaults[9] = {0xA0, 0xA1, 0x00, 0x02, 0x04, 0x01, 0x05, 0x0D, 0x0A};

volatile DMAStatus DMAHandler_B16;
volatile DMAStatus DMAHandler_B17;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_UART4_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void WriteLineChar(UART_HandleTypeDef* huart, char* data){
	/*
	 * Tries to emulate writeline function from matlab. Maintains a timeout of 100 ms.
	 * input: UART_HandleTypeDef* containing a pointer to the huart, char* containing the data to be sent.
	 * */
	if(data == NULL){
		WriteLineChar(huart, " ");
	}
	else{
		HAL_StatusTypeDef status = HAL_UART_Transmit(huart, (uint8_t*)data, strlen(data), HAL_MAX_DELAY);
		if(status !=HAL_OK){
			WriteLineChar(&huart2, "\r\nERROR: WriteLineChar Data Transmission failed\r\n");
		}
		if(status == HAL_BUSY){
			WriteLineChar(&huart2, "\r\nERROR: UART busy\r\n");
		}
	}
	return;
}

void WriteLineUint(UART_HandleTypeDef* huart, uint8_t_array info){
	/*--EXPERIMENTAL--
	 * Tries to emulate writeline function from matlab. Maintains a timeout of 100 ms.
	 * input: UART_HandleTypeDef* containing a pointer to the huart, uint8_t_array containing the data to be sent.
	 * */

	if(info.data == NULL || info.length == 0){
		WriteLineChar(huart, " ");
	}
	else{
		if(HAL_UART_Transmit(huart, info.data, info.length, HAL_MAX_DELAY)!=HAL_OK){

			WriteLineChar(&huart2, "\r\nERROR: WriteLineUint Data Transmission failed\r\n");

		}
	}
	return;
}

const char* FRESULT_String(FRESULT res) {
    switch (res) {
        case FR_OK:                  return "FR_OK";                   // (0) Succeeded
        case FR_DISK_ERR:            return "FR_DISK_ERR";             // (1) A hard error occurred in the low-level disk I/O layer
        case FR_INT_ERR:             return "FR_INT_ERR";              // (2) Assertion failed
        case FR_NOT_READY:           return "FR_NOT_READY";            // (3) The physical drive cannot work
        case FR_NO_FILE:             return "FR_NO_FILE";              // (4) Could not find the file
        case FR_NO_PATH:             return "FR_NO_PATH";              // (5) Could not find the path
        case FR_INVALID_NAME:        return "FR_INVALID_NAME";         // (6) The path name format is invalid
        case FR_DENIED:              return "FR_DENIED";               // (7) Access denied due to prohibited access or directory full
        case FR_EXIST:               return "FR_EXIST";                // (8) Access denied due to file already existing
        case FR_INVALID_OBJECT:      return "FR_INVALID_OBJECT";       // (9) The file/directory object is invalid
        case FR_WRITE_PROTECTED:     return "FR_WRITE_PROTECTED";      // (10) The physical drive is write protected
        case FR_INVALID_DRIVE:       return "FR_INVALID_DRIVE";        // (11) The logical drive number is invalid
        case FR_NOT_ENABLED:         return "FR_NOT_ENABLED";          // (12) The volume has no work area
        case FR_NO_FILESYSTEM:       return "FR_NO_FILESYSTEM";        // (13) There is no valid FAT volume
        case FR_MKFS_ABORTED:        return "FR_MKFS_ABORTED";         // (14) The f_mkfs() aborted due to any problem
        case FR_TIMEOUT:             return "FR_TIMEOUT";              // (15) Could not get a grant to access the volume within the defined period
        case FR_LOCKED:              return "FR_LOCKED";               // (16) The operation is rejected according to the file sharing policy
        case FR_NOT_ENOUGH_CORE:     return "FR_NOT_ENOUGH_CORE";      // (17) LFN working buffer could not be allocated
        case FR_TOO_MANY_OPEN_FILES: return "FR_TOO_MANY_OPEN_FILES";  // (18) Number of open files > _FS_SHARE
        case FR_INVALID_PARAMETER:   return "FR_INVALID_PARAMETER";    // (19) Given parameter is invalid
        default:                     return "Unknown Error";           // Unknown result code
    }
}

//Switch:
void Antenna_Switch(int op){
    /*Sets the antenna 2-in and 4-in switches according to table 4 on Pau's TFG:
    inputs: op, a char* that can be: "RO_FRONT" to configure RO using front antennas.
    "RO_BEHIND" that does the same for the antennas looking behind.
    "REFLECTOMETRY" configurations for GNSS-R experiments
    "CHANGES_POLARIZATION" that is used to study changes in forward signal polarization.*/
    /*Set the correct GPIOx GPIO_Pin once we have PCB layout!!*/
    //Reset all pins to avoid unexpected behaviours in the PCB.
    //We assume for the moment that PC13 is control 1 and PC14 is control 2

    uint16_t control_2_1 = GPIO_PIN_13, control_2_2 = GPIO_PIN_14;
    uint16_t control_4_1 = GPIO_PIN_2, control_4_2 = GPIO_PIN_10, control_4_3 = GPIO_PIN_11; //PLACE THEM IN DEFINE OR HIGH LEVEL!!

    //Reset 2-in switch:
    HAL_GPIO_WritePin(GPIOC, control_2_1, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, control_2_2, GPIO_PIN_RESET);
    //Reset 4-in switch:
    //We assume for the moment that PB2 is control 1, PB10 is control 2, PB11 is control 3.

    HAL_GPIO_WritePin(GPIOB, control_4_2, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, control_4_1, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, control_4_3, GPIO_PIN_SET);

    switch (op){
    case RO_FRONT_RHCP:
    	//TODO: Check this with AC!!
		//Set the 2-in switch to Navigation UP
		HAL_GPIO_WritePin(GPIOC, control_2_1, GPIO_PIN_SET);
		//Set the 4-in switch to Front antenna (RHCP)
		HAL_GPIO_WritePin(GPIOB, control_4_1, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, control_4_2, GPIO_PIN_SET);
		break;

    case RO_FRONT_LHCP:
    	//TODO: Check this with AC!!
		//Set the 2-in switch to Navigation UP
		HAL_GPIO_WritePin(GPIOC, control_2_1, GPIO_PIN_SET);
		//Set the 4-in switch to Front antenna (LHCP)
		HAL_GPIO_WritePin(GPIOB, control_4_3, GPIO_PIN_SET);
		break;

    case RO_BEHIND:
    	//Set the 2-in switch to Navigation UP
		HAL_GPIO_WritePin(GPIOC, control_2_1, GPIO_PIN_SET);
		//Set the 4-in switch to Back antenna
		HAL_GPIO_WritePin(GPIOB, control_4_1, GPIO_PIN_SET);
		break;

    case REFLECTOMETRY:
    	 //Set the 2-in switch to Navigation UP
		HAL_GPIO_WritePin(GPIOC, control_2_1, GPIO_PIN_SET);
		//Set the 4-in switch to Down antenna
		HAL_GPIO_WritePin(GPIOB, control_4_2, GPIO_PIN_SET);
		break;

    case CHANGES_POLARIZATION:
    	 //Set the 2-in switch to Front antenna (LHCP)
		HAL_GPIO_WritePin(GPIOC, control_2_2, GPIO_PIN_SET);
		//Set the 4-in switch to Front antenna (RHCP)
		HAL_GPIO_WritePin(GPIOB, control_4_1, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, control_4_2, GPIO_PIN_SET);

    default:
    	return;
    }
 return;
}
// Get RAW data functions:

uint32_t_array Get_Dopplers_RAW(uint8_t_array raw){
    uint8_t nmeas = raw.data[17];
    uint32_t_array doppler;
    doppler.length = nmeas;
    doppler.data = (uint32_t*)malloc(doppler.length*sizeof(uint32_t));
    if(doppler.data == NULL){exit(-1);}

    int i = 0, j = 0;
    for(i = 0; i<nmeas; i++){
        doppler.data[i] = 0;
        for(j=0; j<4; j++){

            doppler.data[i] |= (uint32_t) raw.data[38+31*i+j]<<(8*(3-j));
        }
    }
    return doppler;
}

uint32_t Get_TOW_RAW(uint8_t_array raw){
/*Gets the time of week (TOW) from the raw message*/
    uint32_t tow = 0x00000000;
    int j = 0;

    for(j=0; j<4; j++){
        tow |= (uint32_t) raw.data[9+j]<<(8*(3-j));

    }

    return tow;
}

uint16_t Get_WN_RAW(uint8_t_array raw){
    uint16_t wn = 0x0000;

    int j = 0;

    for(j=0; j<2; j++){
        wn |= (uint32_t) raw.data[7+j]<<(8*(1-j));
    }
    return wn;
}

uint64_t_array Get_Carrier_Cycle_RAW(uint8_t_array raw){
    uint8_t nmeas = raw.data[17];
    uint64_t_array carrier_cycle;
    carrier_cycle.length = nmeas;
    carrier_cycle.data = (uint64_t*)malloc(nmeas*sizeof(uint64_t));
    if(carrier_cycle.data == NULL){exit(-1);}
    int i = 0, j=0;
    for(i = 0; i<nmeas; i++){
        carrier_cycle.data[i] = 0;
        for(j=0; j<8; j++){
            carrier_cycle.data[i] |= (uint64_t) raw.data[30+31*i+j]<<(8*(7-j));
        }
    }

    return carrier_cycle;
}

uint64_t_array Get_Pseudorange_RAW(uint8_t_array raw){
    uint8_t nmeas = raw.data[17];
    uint64_t_array pseudorange;
    pseudorange.length = nmeas;
    pseudorange.data = (uint64_t*)malloc(nmeas*sizeof(uint64_t));
    if(pseudorange.data == NULL){exit(-1);}
    int i = 0, j=0;
    for(i = 0; i<nmeas; i++){
        pseudorange.data[i] = 0;
        for(j=0; j<8; j++){

            pseudorange.data[i] |= (uint64_t) raw.data[22+31*i+j]<<(8*(7-j));
        }
    }

    return pseudorange;
}

uint8_t_array Get_CN0_RAW(uint8_t_array raw){
    uint8_t nmeas = raw.data[17];
    uint8_t_array cno;
    cno.length = nmeas;
    cno.data = (uint8_t*)malloc(cno.length*sizeof(uint8_t));
    for(int i=0; i<nmeas; i++){
        cno.data[i] = raw.data[21+31*i];
    }

    return cno;
}

char** Get_Constellation_RAW(uint8_t_array raw){
    /*Obtains the constellation of the GNSS satellite from the raw message. (WARNING: in AN0039, the value 0xXY is in little endian, so X is the frequency band and Y is the constellation!!)
    input: raw message in an uint8_t array.
    output: char** matrix of length nmeas with each char* containing the constellation: GPS, SBA (SBAS), GLO (GLONASS), GAL (Galileo), BEI (BEIDOU), QZS (QZSS), IRN (IRNSS)*/
    uint8_t nmeas = raw.data[17];
    int i = 0;
    uint8_t buffer;
    char** constellation = (char**)malloc(nmeas*sizeof(char*));
    if(constellation==NULL){exit(-1);}
    for(i=0; i<nmeas; i++){
        constellation[i] = (char*)malloc(4*sizeof(char));
        if(constellation[i]==NULL){exit(-1);}
        constellation[i][3] = '\0';
    }
    for(i=0; i<nmeas; i++){
        buffer = raw.data[18+31*i];
        strcpy(constellation[i], num_to_constellation[(buffer) & 0x0F]);

    }
    return constellation;

}

char** Get_Frequency_Band_RAW(uint8_t_array raw, char** constellation){
    /*Obtains the frequency band of the signal from the GNSS satellite taken from the raw message. (WARNING: in AN0039, the value 0xXY is in little endian, so X is the frequency band and Y is the constellation!!)
    inputs: raw message in an uint8_t array, char** containing the constellations (call Get_Constellation_RAW first!!).
    output: char** matrix of length nmeas with each char* containing the frequency bands (see AN0039 for detail or the code below).
    */
    uint8_t nmeas = raw.data[17];
    int i = 0;
    uint8_t buffer;
    char** freq_band = (char**)malloc(nmeas*sizeof(char*));
    if(freq_band==NULL){exit(-1);}
    for(i=0; i<nmeas; i++){
        freq_band[i] = (char*)malloc(6*sizeof(char));
        if(freq_band[i]==NULL){exit(-1);}
        freq_band[i][5] = '\0';
    }
    for(i=0; i<nmeas; i++){
        buffer = (raw.data[18+31*i]>>4) & 0x0F;

        if(strcmp(constellation[i], "GPS")==0 || strcmp(constellation[i], "QZS")==0){
            switch(buffer){
                case 0:
                strcpy(freq_band[i], "L1C/A");
                break;
                case 1:
                strcpy(freq_band[i], "L1C");
                break;
                case 2:
                strcpy(freq_band[i], "L2C");
                break;
                case 4:
                strcpy(freq_band[i], "L5");
                break;
                case 6:
                strcpy(freq_band[i], "LEX");
                break;
            }
        }
        else if(strcmp(constellation[i], "GLO")==0){
            switch(buffer){
                case 0:
                strcpy(freq_band[i], "L1");
                break;
                case 2:
                strcpy(freq_band[i], "L2");
                break;
                case 4:
                strcpy(freq_band[i], "L3");
                break;
            }

        }
        else if(strcmp(constellation[i], "GAL")==0){
            switch(buffer){
                case 0:
                strcpy(freq_band[i], "E1");
                break;
                case 4:
                strcpy(freq_band[i], "E5a");
                break;
                case 5:
                strcpy(freq_band[i], "E5b");
                break;
                case 6:
                strcpy(freq_band[i], "E6");
                break;

            }

        }

        else if(strcmp(constellation[i], "BEI")==0){
            switch(buffer){
                case 0:
                strcpy(freq_band[i], "B1I");
                break;
                case 1:
                strcpy(freq_band[i], "B1C");
                break;
                case 4:
                strcpy(freq_band[i], "B2A");
                break;
                case 5:
                strcpy(freq_band[i], "B2I");
                break;
                case 7:
                strcpy(freq_band[i], "B3I");
                break;

            }
        }
        else if(strcmp(constellation[i], "SBA")==0){
            strcpy(freq_band[i], "L1");

        }
        else{
            strcpy(freq_band[i], "L5");
        }
    }
    return freq_band;

}

uint8_t_array Get_SVID_RAW(uint8_t_array raw){
    /*Gets SVIDs from raw message.
    input: uint8_t_array containing the raw message.
    output: uint8_t_array containing the SVID for each channel.*/
    uint8_t_array SVID;
    SVID.length = raw.data[17];
    SVID.data = malloc(SVID.length*sizeof(uint8_t));
    if(SVID.data==NULL){exit(-1);}
    int i = 0;
    for(i=0; i<SVID.length; i++){
        SVID.data[i] = raw.data[19+i*31];
    }
    return SVID;
}

uint8_t_array Tokenize_uint8_t_array(uint8_t_array arr, int* beginning_mark){
    /*Tokenizes a buffer containing messages in binary protocol for B16/17 receivers.
    inputs: a uint8_t_array containing the buffer, an int* pointer storing the beginning point of the message we're looking for.
    beginning_mark updates itself automatically. Beware of resetting it after the buffer is complete!!
    outputs: the message.
    */
    int i = 0, j = 0;
    uint8_t_array msg = {NULL, 0};
    //Get the points at which the message begins and ends.
    if(*beginning_mark < arr.length -2){
        for(i=*beginning_mark; i<arr.length-1; i++){
            if(arr.data[i]==0xA0 && arr.data[i+1] == 0xA1){
                for(j=i+2; j<arr.length; j++){
                    if(arr.data[j-1]==0x0D && arr.data[j]==0x0A){
                        msg.length = j+1-i;
                        msg.data = (uint8_t*)malloc(msg.length*sizeof(uint8_t));
                        if(msg.data == NULL){exit(-1);}
                        memcpy(msg.data, arr.data+i, msg.length);
                        *beginning_mark = j+1;
                        return msg;
                    }
                }

            }

        }
    }

    // If no valid message is found, return an empty array

    *beginning_mark = arr.length;
    return msg;
}

char* Hex_to_String(uint8_t_array arr){
    /*Converts a hexadecimal array to a string (without spaces).
    inputs: uint8_t_array with the array to convert.
    output: char* with the converted array to a string.*/
    int i = 0;
    char* str = (char*)malloc((2*arr.length+1)*sizeof(char));

    char* buffer = (char*)malloc(3*sizeof(char));
    if(str==NULL||buffer==NULL){exit(-1);}
    str[2*arr.length] = '\0';
    buffer[2] = '\0';

    for(i=0; i<arr.length; i++){
        char hexChars[17]="0123456789ABCDEF";
        buffer[0] = hexChars[(arr.data[i] >> 4) & 0x0F]; // High nibble
        buffer[1] = hexChars[arr.data[i] & 0x0F];        // Low nibble

        strcpy(str+2*i, buffer);

    }
    free(buffer);
    return str;
}

uint8_t_array Build_Binary_OBC_Data(uint8_t_array raw, char* receiver_id, uint16_t wn, uint32_t tow, uint8_t_array svid, uint32_t_array doppler, uint64_t_array pseudorange, uint64_t_array carrier, uint8_t_array cno){
    /*Builds the binary message containing all the raw data that will be sent to the OBC. It's 23*NMEAS+13 bytes long. */
    uint8_t_array msg;
    uint16_t nmeas = raw.data[17];
    msg.length = 23*nmeas+13;
    msg.data = (uint8_t*)malloc(msg.length*sizeof(uint8_t));
    if(msg.data == NULL){exit(-1);}
    int i= 0, j = 0;
    uint8_t buffer = 0x00;

    msg.data[0] = 0xA0, msg.data[1] = 0xA1;
    msg.data[msg.length-2] = 0x0D, msg.data[msg.length-1] = 0x0A;

    if(strcmp(receiver_id, "B16")==0){msg.data[2] = 0x01;}
    if(strcmp(receiver_id, "B17")==0){msg.data[2] = 0x11;}

    memcpy(msg.data+3, &wn, 2);
    for(j=0; j<4; j++){
        buffer = (tow>>(24-8*j))&0xFF;
        memcpy(msg.data+5+j, &buffer, 1);
    }
    memcpy(msg.data+9, &nmeas, 2);

    for(i=0; i<nmeas; i++){
        //Constellation + Frequency
        memcpy(msg.data+10+23*i, raw.data+18+(i*31), 1);
        //SVID
        memcpy(msg.data+11+23*i, raw.data+19+(i*31), 1);

        for(j=0; j<4; j++){
            buffer = (doppler.data[i]>>(24-8*j)) & 0xFF;
            memcpy(msg.data+12+j+23*i, &buffer, 1);
        }
        for(j=0; j<8; j++){
            buffer = (pseudorange.data[i]>>(56-8*j)) & 0xFF;
            memcpy(msg.data+16+j+23*i, &buffer, 1);
        }
        for(j=0; j<8; j++){
            buffer = (carrier.data[i]>>(56-8*j)) & 0xFF;
            memcpy(msg.data+24+j+23*i, &buffer, 1);
        }

        memcpy(msg.data+32+23*i, &cno.data[i], 1);
    }

    uint8_t checksum = 0x00;

    for(i=2; i<msg.length-3;i++){
        checksum ^= msg.data[i];
    }
    msg.data[msg.length-3] = checksum;
    return msg;
}

void Process_Binary_Data(uint8_t_array arr, char* receiver_id, FIL fil){

	int beginning_mark = 0;

	while(beginning_mark < arr.length){

		uint8_t_array msg = Tokenize_uint8_t_array(arr, &beginning_mark);

		if(msg.data[4]==0xE5){

			uint8_t nmeas = msg.data[17]; //For future conversion towards static memory...

			uint32_t tow = Get_TOW_RAW(msg);

			uint16_t wn = Get_WN_RAW(msg);

			uint8_t_array cno_msg = Get_CN0_RAW(msg);

			uint64_t_array carrier_msg = Get_Carrier_Cycle_RAW(msg);

			uint64_t_array pseudo_msg = Get_Pseudorange_RAW(msg);

			uint32_t_array doppler_msg = Get_Dopplers_RAW(msg);

			uint8_t_array svid_msg = Get_SVID_RAW(msg);

			char** constellation_msg = Get_Constellation_RAW(msg);

			char** freq_band_msg = Get_Frequency_Band_RAW(msg, constellation_msg);

			//TODO: add SRAM/SD storage and OBC transmit (pass storage as pointer...).

			uint64_t carrier_placeholder = 0, pseudo_placeholder = 0;

			uint32_t doppler_placeholder = 0;


			//Craft data entry: WN, TOW, Constellation, SVID, Freq Band, Lat, Lon, Alt, Doppler, Pseudorange, Carrier, CN0\n

			float doppler_float = 0;
			double pseudorange_double = 0, carrier_double = 0;

			for(int i=0; i<cno_msg.length; i++){

				doppler_float = 0;
				pseudorange_double = 0;
				carrier_double = 0;

				char* msg_buffer = malloc(401*sizeof(char));

				if(msg_buffer==NULL){exit(-1);}


				msg_buffer[400] = '\0';

				//convert from hex to several types.

				doppler_placeholder = doppler_msg.data[i];

				pseudo_placeholder = pseudo_msg.data[i];

				carrier_placeholder = carrier_msg.data[i];

				memcpy(&doppler_float, &doppler_placeholder, sizeof(float));

				memcpy(&pseudorange_double, &pseudo_placeholder, sizeof(double));

				memcpy(&carrier_double, &carrier_placeholder, sizeof(double));

				snprintf(msg_buffer, 400, "%s,%u,%u,%s,%u,%s,,,,%f,%lf,%lf,%u\n", receiver_id, wn, tow, constellation_msg[i], svid_msg.data[i], freq_band_msg[i], doppler_float, pseudorange_double, carrier_double, cno_msg.data[i]);

				if(SD_card_enabled == 1){
					f_puts(msg_buffer, &fil);
				}


				free(msg_buffer);

			}

			//Craft message for OBC.

			uint8_t_array msg_OBC = Build_Binary_OBC_Data(msg, receiver_id, wn, tow, svid_msg, doppler_msg, pseudo_msg, carrier_msg, cno_msg);


			HAL_UART_Transmit_IT(&UART_OBC, msg_OBC.data, msg_OBC.length);
			free(cno_msg.data), free(carrier_msg.data), free(pseudo_msg.data), free(doppler_msg.data), free(svid_msg.data);

			for(int i=0; i<msg.data[17]; i++){

			free(constellation_msg[i]), free(freq_band_msg[i]);

			}

			free(constellation_msg), free(freq_band_msg), free(msg_OBC.data);

		}


			free(msg.data);

	}

	return;

}
void Process_Binary_Data_Reduced(uint8_t_array arr, char* receiver_id){
	//TESTING PURPOSES ONLY!!!
	int beginning_mark = 0;

	while(beginning_mark < arr.length){
		uint8_t_array msg = Tokenize_uint8_t_array(arr, &beginning_mark);
		if(msg.data != NULL){
			char msg_id_str[50];
			sprintf(msg_id_str, "\r\nMsg ID: %X\r\n", msg.data[4]);
			HAL_UART_Transmit_IT(&huart2,(uint8_t*)msg_id_str, strlen(msg_id_str));
		}
	}
	return;
}
//Detect ACK/NACK:
int Detect_ACK(uint8_t_array buf){
	/*Used in DMA Receive Callback functions to check if there are ACK messages.
	 * Transmits to terminal that an ACK has been received when finding one along the message ID (may find more than one).
	 * inputs: uint8_t_array with the buffer.
	 * outputs: an int that is 1 when an ACK has been found, and 0 when no ACKs have been found.*/
	int ack_flag = 0;
	int i = 0;

	for(i=0; i<buf.length-5; i++){
		if(buf.data[i]==0xA0 && buf.data[i+1]==0xA1 && buf.data[i+4]==0x83){
			ack_flag = 1;
		}
	}
	return ack_flag;
}

int Detect_NACK(uint8_t_array buf){
	/*Used in DMA Receive Callback functions to check if there are NACK messages.
		 * Transmits to terminal that an NACK has been received when finding one along the message ID (may find more than one).
		 * inputs: uint8_t_array with the buffer.
		 * outputs: an int that is 1 when an NACK has been found, and 0 when no NACKs have been found.*/
	int nack_flag = 0;
	int i = 0;

	for(i=0; i<buf.length-5; i++){
		if(buf.data[i]==0xA0 && buf.data[i+1]==0xA1 && buf.data[i+4]==0x84){
			nack_flag = 1;
		}
	}
	return nack_flag;
}

void Process_NMEA_Data(char* buffer_str_cpy){
    /*Function to process the NMEA data, parsing the buffer string into messages and extracting mode, talker ID, UTC time, latitude and longitude
    input: buffer_str_cpy string with the buffer (only when NMEA is set)*/

    char* message;
    while((message = strsep(&buffer_str_cpy, "\n"))!=NULL){
        //139 is not in the Get functions and is inside this loop...
        char* message_cpy=(char*)malloc((strlen(message)+1)*sizeof(char));
        if(message_cpy==NULL){exit(-1);}
        strcpy(message_cpy, message);
        message_cpy[strlen(message)]='\0';

        for(int i=0; i<strlen(message_cpy); i++){if(message_cpy[i]=='\r'){message_cpy[i]='\0';}}

        int is_nmea = is_NMEA(message_cpy);

        if(is_nmea == 1){
            //find the checksum.
            int checksum_computed = Checksum_NMEA(message_cpy);

            char* checksum_message = malloc(3*sizeof(char));
            if(checksum_message == NULL){exit(-1);}
            checksum_message[2] = '\0';
            if(strlen(message_cpy)-2>= 0){
                checksum_message[0] = message_cpy[strlen(message_cpy)-2];
                checksum_message[1] = message_cpy[strlen(message_cpy)-1];
            }else{
                checksum_message[0] = '0';
                checksum_message[1] = '0';
            }

            int checksum_message_int = (int) strtol(checksum_message, NULL, 16);

            if((message_cpy != NULL) && (strlen(message_cpy)>1) && (checksum_message_int-checksum_computed == 0)){
                char* mode = Get_Mode(message);
                char* talker_ID = Get_Talker_ID(message);
                char* utc_date = Get_UTCDate(message, mode);
                char* utc_time = Get_UTCTime(message, mode);
                char* latitude = Get_Latitude(message, mode);
                char* longitude = Get_Longitude(message, mode);
                char* msl_altitude = Get_Altitude(message, mode);


                //Check value for timeout
                char* terminal_pvt = malloc(200*sizeof(char));
                if(terminal_pvt == NULL){exit(-1);}
                snprintf(terminal_pvt, 200, "\r\n%s,%s,%s,%s,%s,%s,%s\r\n", mode, talker_ID, utc_date, utc_time, latitude, longitude, msl_altitude);
                strcpy(pvt_reserve, terminal_pvt);
                HAL_UART_Transmit_IT(&huart2, (uint8_t*)terminal_pvt, strlen(terminal_pvt));

                if(SD_card_enabled == 1){
					fres = f_sync(&filB16);
					if(fres==FR_OK){
						HAL_UART_Transmit_IT(&huart2, (uint8_t*)"File synchronized!\r\n", strlen("File synchronized!\r\n"));
					}
					fres = f_write(&filB16, terminal_pvt, strlen(terminal_pvt), &BW);
					const char* fres_str = FRESULT_String(fres);
					HAL_UART_Transmit_IT(&huart2, (uint8_t*)fres_str, strlen(fres_str));
					if(fres != FR_OK){
						HAL_UART_Transmit_IT(&huart2, (uint8_t*)"Data not stored!\r\n", strlen("Data not stored!\r\n"));
					}
                }

                free(mode), free(talker_ID), free(utc_date), free(utc_time), free(msl_altitude), free(latitude), free(longitude);
                free(terminal_pvt);
            }
            free(checksum_message);
        }
            free(message_cpy);

    }

    return;
}

//Callbacks

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == UART_B16.Instance){
		DMAHandler_B16.stat = DATA_HALF_RECEIVED;
	}

	if(huart->Instance == UART_B17.Instance){
		DMAHandler_B17.stat = DATA_HALF_RECEIVED;
	}

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == UART_B16.Instance){
		DMAHandler_B16.stat = DATA_RECEIVED;
		DMAHandler_B16.request = REQUEST_FULFILLED;
	}
	if(huart->Instance == UART_B17.Instance){
		DMAHandler_B17.stat = DATA_RECEIVED;
		DMAHandler_B17.request = REQUEST_FULFILLED;
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	return;
}

void HAL_UART_RxErrorCallback(UART_HandleTypeDef *huart){
	if(huart->Instance == UART_B16.Instance){
		DMAHandler_B16.stat = DMA_ERROR;
	}
	else if(huart->Instance == UART_B17.Instance){
		DMAHandler_B17.stat = DMA_ERROR;
	}
	char error[50];

	switch(huart->ErrorCode){

		case HAL_UART_ERROR_PE:
			sprintf(error, "\r\nDMA Error: Parity error\r\n");
			break;
		case HAL_UART_ERROR_FE:
			sprintf(error, "\r\nDMA Error: Frame error\r\n");
			break;
		case HAL_UART_ERROR_NONE:
			sprintf(error, "\r\nDMA Error: No error\r\n");
			break;
		case HAL_UART_ERROR_ORE:
			sprintf(error, "\r\nDMA Error: Overrun error\r\n");
			break;
		case HAL_UART_ERROR_DMA:
			sprintf(error, "\r\nDMA Error: DMA transfer error\r\n");
			break;
		default:
			sprintf(error, "\r\nDMA Error: Noise error\r\n");

	}

	HAL_UART_Transmit_IT(&UART_OBC, (uint8_t*)error, strlen(error));
}

void Init_OBC_Cmd(OBC_cmd* OBC){
	OBC->change_output_message_format_B16 = 1, OBC->change_output_message_format_B17 = 0;
	OBC->change_power_mode_B16 = 0, OBC->change_power_mode_B17 = 0, OBC->change_switch_mode = 0;
	OBC->doze_B16 = 0, OBC->doze_B17 = 0, OBC->read_from_SD_card = 0;
	OBC->reset_B16 = 0, OBC->reset_B17 = 0, OBC->set_output_B16 = NMEA, OBC->set_output_B17 = 0;
	OBC->set_switch = 0, OBC->status_report_B16 = 0, OBC->status_report_B17 = 0;
	return;
}

void Execute_OBC_Cmd(OBC_cmd* OBC, receiver_status* B16_status, receiver_status* B17_status){
	  //Set power mode.
	  if(OBC->change_power_mode_B16 == 1){
		  if(OBC->doze_B16 == 1){
			  HAL_UART_Transmit_IT(&UART_B16, enter_doze_mode, 9);
			  OBC->doze_B16 = 0, B16_status->receiver_dozed = 1;
		  }
		  if(OBC->reset_B16 == 1){
			  //TODO: Obtain values for day, month, year, hour, minute, second, lat, lon, alt.
			  uint8_t_array reset_cmd = Receiver_Cold_Restart(COLD_START, 01, 01, 2025, 00, 00, 00, 0, 0, 0);
			  HAL_UART_Transmit_IT(&UART_B16, reset_cmd.data, reset_cmd.length);
			  free(reset_cmd.data);
			  OBC->reset_B16 = 0, B16_status->receiver_dozed = 0, B16_status->receiver_on = 1;

		  }
		  OBC->change_power_mode_B16 = 0;
	  }
	  if(OBC->change_power_mode_B17 == 1){
		  if(OBC->doze_B17 == 1){
			  HAL_UART_Transmit_IT(&UART_B17, enter_doze_mode, 9);
			  cmd_sent_B17 = 1;

			  OBC->doze_B17 = 0, B17_status->receiver_dozed = 1;

		  }
		  if(OBC->reset_B17 == 1){
			  //TODO: Obtain values for day, month, year, hour, minute, second, lat, lon, alt.
			  uint8_t_array reset_cmd = Receiver_Cold_Restart(COLD_START, 01, 01, 2025, 00, 00, 00, 0, 0, 0);
			  HAL_UART_Transmit_IT(&UART_B17, reset_cmd.data, reset_cmd.length);

			  free(reset_cmd.data);
			  OBC->reset_B17 = 0, B17_status->receiver_dozed = 0, B17_status->receiver_on = 1;

		  }
		  OBC->change_power_mode_B17 = 0;
	  }
	  //Set antenna switches.
	  if(OBC->change_switch_mode == 1){
		  Antenna_Switch(OBC->set_switch);
		  OBC->change_switch_mode = 0;
	  }

	  //Set receiver output message format (NMEA or binary)
	  if(OBC->change_output_message_format_B16 == 1){
		  if(OBC->set_output_B16 == NMEA){
			  //transmit cmd_change_message_format_to_NMEA through appropriate UART
			  uint8_t_array cmd_change_message_format_to_NMEA = Change_Message_Format("NMEA", UPDATE_TO_SRAM);
			  HAL_UART_Transmit_IT(&UART_B16, cmd_change_message_format_to_NMEA.data, cmd_change_message_format_to_NMEA.length);
			  free(cmd_change_message_format_to_NMEA.data);
			  B16_status->NMEA_set = 1;
		  }
		  else{
			  //transmit cmd_change_message_format_to_binary through appropriate UART
			  uint8_t_array cmd_change_message_format_to_binary = Change_Message_Format("Binary", UPDATE_TO_SRAM);
			  HAL_UART_Transmit_IT(&UART_B16, cmd_change_message_format_to_binary.data, cmd_change_message_format_to_binary.length);
			  free(cmd_change_message_format_to_binary.data);
			  B16_status->binary_set = 1;
		  }

		  OBC->change_output_message_format_B16 = 0;
	  }

	  if(OBC->change_output_message_format_B17==1){
		  if(OBC->set_output_B17 == NMEA){
			  uint8_t_array configure_rtk_mode = Configure_RTK_Mode_and_Operational_Function(RTK_ROVER, RTK_ROVER_NORMAL, 0,0,0,0,0,0,UPDATE_TO_SRAM);
			  HAL_UART_Transmit_IT(&UART_B17, configure_rtk_mode.data, configure_rtk_mode.length);
			  free(configure_rtk_mode.data);

			  B17_status->NMEA_set = 1;
			  B17_status->binary_set = 0;
		  }
		  else if(OBC->set_output_B17 == BINARY){
			  uint8_t_array configure_rtk_mode = Configure_RTK_Mode_and_Operational_Function(RTK_BASE, RTK_BASE_KINEMATIC, 0x00010000,0x03,0,0,0,0,UPDATE_TO_SRAM);
			  HAL_UART_Transmit_IT(&UART_B17, configure_rtk_mode.data, configure_rtk_mode.length);
			  uint8_t_array configure_binary_output = Configure_Binary_Measurement_Data_Output(8, 0, 0, 0, 0, 0, 1, UPDATE_TO_SRAM);
			  HAL_UART_Transmit_IT(&UART_B17, configure_binary_output.data, configure_rtk_mode.length);
			  free(configure_rtk_mode.data), free(configure_binary_output.data);

			  B17_status->binary_set = 1;
			  B17_status->NMEA_set = 0;
			  B17_status->ext_raw_data_set = 1;
		  }

		  OBC->change_output_message_format_B17 = 0;
	  }

	  return;
}

//MISC:


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	uint8_t_array buffer_B16_arr;
	uint8_t_array buffer_B17_arr;
	uint8_t current_buffer_B16[BUFFER_SIZE_B16];
	uint8_t current_buffer_B17[BUFFER_SIZE_B17];

	buffer_B16_arr.length = BUFFER_SIZE_B16;
	buffer_B17_arr.length = BUFFER_SIZE_B17;

	buffer_B16_arr.data = current_buffer_B16;
	buffer_B17_arr.data = current_buffer_B17;

	DMAHandler_B16.stat = IDLE;
	DMAHandler_B16.request = REQUEST_DATA;
	DMAHandler_B17.stat = IDLE;
	DMAHandler_B17.request = REQUEST_DATA;

    int beginning_mark_B16 = 0;
    int beginning_mark_B17 = 0;
    int iteration_index = 0;
    uint8_t_array OBC_Rx_arr;
    OBC_Rx_arr.length = 50;
    OBC_Rx_arr.data = malloc(OBC_Rx_arr.length*sizeof(uint8_t));
    if(OBC_Rx_arr.data == NULL){exit(-1);}

    //Receiver status:
    receiver_status B16_status, B17_status;
    B16_status.NMEA_set = 1, B16_status.binary_set = 0, B16_status.ext_raw_data_set = 0, B16_status.receiver_on = 1, B16_status.receiver_dozed = 0;
    B17_status.NMEA_set = 0, B17_status.binary_set = 1, B17_status.ext_raw_data_set = 1, B17_status.receiver_on = 1, B17_status.receiver_dozed = 0;

    //OBC command object:
    OBC_cmd OBC;
    //Configurations switch:
    Init_OBC_Cmd(&OBC);

    //commands:

    uint8_t_array cmd_configure_navigation_data_message_interval = Configure_Navigation_Data_Message_Interval(3, UPDATE_TO_SRAM);

    uint8_t query_software_version[9] = {0xA0, 0xA1, 0x00, 0x02, 0x02, 0x00, 0x02, 0x0D, 0x0A};
    uint8_t_array cmd_query_software_version = Build_uint8_t_Command(query_software_version, 9);

    uint8_t query_power_mode[8] = {0xA0, 0xA1, 0x00, 0x01, 0x15, 0x15, 0x0D, 0x0A};
    uint8_t_array cmd_query_power_mode = Build_uint8_t_Command(query_power_mode, 8);

    uint8_t query_nav_data_msg_int[9] = {0xA0, 0xA1, 0x00, 0x02, 0x64, 0x30, 0x54, 0x0D, 0x0A};
    uint8_t_array cmd_query_navigation_data_msg_interval = Build_uint8_t_Command(query_nav_data_msg_int, 9);

    uint8_t set_pwr_to_normal[10] = {0xA0, 0xA1, 0x00, 0x03, 0x0C, 0x00, 0x00, 0x0C, 0x0D, 0x0A};
    uint8_t_array cmd_set_power_to_normal = Build_uint8_t_Command(set_pwr_to_normal, 10);

    uint8_t NMEA_intervals[13] = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint8_t_array cmd_configure_NMEA_intervals = Configure_NMEA_Message_Interval(NMEA_intervals);
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
  MX_SPI1_Init();
  MX_UART4_Init();
  MX_FATFS_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Transmit_IT(&UART_OBC, (uint8_t*)"\r\nNew run\r\n", strlen("\r\nNew run\r\n"));
  //Initialize Micro SD module


  if(SD_card_enabled == 1){
	  FRESULT fres = f_mount(&fs, "", 1);

	  if(fres == FR_OK){
		  HAL_UART_Transmit_IT(&UART_OBC, (uint8_t*)"\r\nSD card mounted\r\n", strlen("\r\nSD card mounted\r\n"));
	  }
	  //Mount/unmount SD correct, no problems here...
	  char micro_sd_str_0[35];
	  sprintf(micro_sd_str_0, "fres code f_mount: %s\r\n",FRESULT_String(fres));
	  WriteLineChar(&UART_OBC, micro_sd_str_0);

	  HAL_Delay(1000);
	  DSTATUS stat = disk_status(0);
	  if(stat != RES_OK){
		  WriteLineChar(&UART_OBC, "ERROR: Disk not ready after mount\r\n");
	  }
	  //Check available space...
	  DWORD fre_clust;
	  FATFS *pfs;
	  uint32_t totalSpace, freeSpace;
	  fres = f_getfree("", &fre_clust, &pfs);
	  if(fres == FR_OK){
		  totalSpace = (uint32_t)((pfs->n_fatent - 2) * pfs->csize * 0.5);
		  freeSpace = (uint32_t)(fre_clust * pfs->csize * 0.5);
		  char space[200];
		  sprintf(space,"TotalSpace : %lu bytes, FreeSpace = %lu bytes\r\n", totalSpace, freeSpace);
		  HAL_UART_Transmit_IT(&UART_OBC, (uint8_t*) space, strlen(space));
	  }

	  char micro_sd_str_1[30];
	  sprintf(micro_sd_str_1, "fres code f_getfree: %s\r\n",FRESULT_String(fres));
	  HAL_UART_Transmit_IT(&UART_OBC, (uint8_t*)micro_sd_str_1, strlen(micro_sd_str_1));

	  fres = f_open(&filB16, "B16_data.txt", FA_WRITE | FA_READ | FA_CREATE_ALWAYS);
	  if(fres == FR_OK){
		  HAL_UART_Transmit_IT(&UART_OBC, (uint8_t*)"file opened successfully!\r\n", strlen("file opened successfully!\r\n"));
	  }

	  fres = f_puts("Una vez vi una vaca vestida de uniforme\r\n", &filB16);

	  if(fres==FR_OK){
		  HAL_UART_Transmit_IT(&UART_OBC, (uint8_t*)"write successful\r\n", strlen("write successful\r\n"));
	  }
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  //Receive data from B16/17 and store into buffers:

  if(B16_ENABLED){
	  HAL_UART_Receive_DMA(&UART_B16, buffer_B16, 2*BUFFER_SIZE_B16);
  }

  if(B17_ENABLED){
	  HAL_UART_Receive_DMA(&UART_B17, buffer_B17, 2*BUFFER_SIZE_B17);
  }

  while (1)
  {

	  if(B17_ENABLED){

		  char* buffer_full_str = malloc(50*sizeof(char));

		  if(DMAHandler_B17.stat == DATA_HALF_RECEIVED){
			  memcpy(current_buffer_B17, buffer_B17, BUFFER_SIZE_B17);
			  sprintf(buffer_full_str,"\r\nbuffer full B17 1\r\n");
		  }
		  else if(DMAHandler_B17.stat == DATA_RECEIVED){
			  memcpy(current_buffer_B17, buffer_B17+BUFFER_SIZE_B17, BUFFER_SIZE_B17);
			  sprintf(buffer_full_str,"\r\nbuffer full B17 2\r\n");
			  DMAHandler_B17.request = REQUEST_DATA;
		  }
		  if(DMAHandler_B17.stat == DATA_RECEIVED || DMAHandler_B17.stat == DATA_HALF_RECEIVED){
			  HAL_UART_Transmit_IT(&UART_OBC, (uint8_t*)buffer_full_str, strlen(buffer_full_str));
			  free(buffer_full_str);
			  if(B17_status.binary_set){
				  Process_Binary_Data_Reduced(buffer_B17_arr, "B17");
			  }
			  else{
				  char* buffer_B17_str = malloc((buffer_B17_arr.length+1)*sizeof(char));
				  if(buffer_B17_str == NULL){exit(-1);}
				  buffer_B17_str[buffer_B17_arr.length] = '\0';
				  memcpy(buffer_B17_str, buffer_B17_arr.data, buffer_B17_arr.length);
				  Process_NMEA_Data(buffer_B17_str);
				  free(buffer_B17_str);
			  }

			  DMAHandler_B17.stat = IDLE;
	  	  }

	  }

	  if(B16_ENABLED){

	 		  char* buffer_full_str = malloc(50*sizeof(char));

	 		  if(DMAHandler_B16.stat == DATA_HALF_RECEIVED){
	 			  memcpy(current_buffer_B16, buffer_B16, BUFFER_SIZE_B16);
	 			  sprintf(buffer_full_str,"\r\nbuffer full B16 1\r\n");
	 		  }
	 		  else if(DMAHandler_B16.stat == DATA_RECEIVED){
	 			  memcpy(current_buffer_B16, buffer_B16+BUFFER_SIZE_B16, BUFFER_SIZE_B16);
	 			  sprintf(buffer_full_str,"\r\nbuffer full B16 2\r\n");
	 			  DMAHandler_B16.request = REQUEST_DATA;
	 		  }
	 		  if(DMAHandler_B16.stat == DATA_RECEIVED || DMAHandler_B16.stat == DATA_HALF_RECEIVED){
	 			  HAL_UART_Transmit_IT(&UART_OBC, (uint8_t*)buffer_full_str, strlen(buffer_full_str));
	 			  free(buffer_full_str);
	 			  if(B16_status.binary_set){
	 				  Process_Binary_Data_Reduced(buffer_B17_arr, "B17");
	 			  }
	 			  else{
	 				  char* buffer_B16_str = malloc((buffer_B16_arr.length+1)*sizeof(char));
	 				  if(buffer_B16_str == NULL){exit(-1);}
	 				  buffer_B16_str[buffer_B16_arr.length] = '\0';
	 				  memcpy(buffer_B16_str, buffer_B16_arr.data, buffer_B16_arr.length);
	 				  Process_NMEA_Data(buffer_B16_str);
	 				  free(buffer_B16_str);
	 			  }

	 			  DMAHandler_B16.stat = IDLE;
	 	  	  }

	 	  }

	  //Add OBC command listener

	  Execute_OBC_Cmd(&OBC, &B16_status, &B17_status);

	  if((B16_status.NMEA_set == 1) && (iteration_index%100000 == 0)){
		  HAL_UART_Transmit_IT(&huart4, cmd_configure_NMEA_intervals.data, cmd_configure_NMEA_intervals.length);
	  }

	  iteration_index++;
	  if(iteration_index > 10000000){
		  break;
	  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  if(SD_card_enabled == 1){
	  //Close files and unmount SD card.
	  if(f_close(&filB16) == FR_OK){
	  		  WriteLineChar(&UART_OBC, "File B16 closed\r\n");
	  	  }
	  else{
	  		  WriteLineChar(&UART_OBC, "ERROR: File B16 not closed\r\n");
	  	  }
	  if(f_close(&filB17) == FR_OK){
		  WriteLineChar(&UART_OBC, "File B17 closed\r\n");
	  }
	  else{
		  WriteLineChar(&UART_OBC, "ERROR: File B17 not closed\r\n");
	  }

	  f_mount(NULL, "", 1);

  }

  free(OBC_Rx_arr.data), free(cmd_configure_navigation_data_message_interval.data);
  free(cmd_query_software_version.data);
  return 0;
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA2_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel5_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SD_CS_Pin|GPIO_PIN_2|GPIO_PIN_10|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : SD_CS_Pin PB2 PB10 PB11 */
  GPIO_InitStruct.Pin = SD_CS_Pin|GPIO_PIN_2|GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
