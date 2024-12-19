#include<stdint.h>
#include<string.h>
#include<stdio.h>
#include<stdlib.h>

//Receiver Start Modes:
#define COLD_START				0x03
#define WARM_START				0x02
#define HOT_START				0x01
//Antenna switch modes:
#define RO_FRONT_RHCP			0x01
#define RO_FRONT_LHCP			0x02
#define RO_BEHIND				0x03
#define REFLECTOMETRY			0x04
#define CHANGES_POLARIZATION	0x05

//Output protocols for B16:
#define NMEA					0x01
#define BINARY					0x02
#define NO						0x00

//Options for DMAStatus object:
//DMAStatus.request:
#define REQUEST_DATA					0x00
#define REQUEST_FULFILLED				0x0F
#define REQUEST_SENT					0x05
//DMAStatus.stat:
#define DATA_HALF_RECEIVED				0x01
#define DATA_RECEIVED					0x02
#define NO_DATA_AVAILABLE				0xFF
#define DMA_ERROR			      		0xF0
#define IDLE							0x00

typedef struct uint8_t_array{
	uint8_t* data;
	uint16_t length;
}uint8_t_array;

typedef struct uint16_t_array{
	uint16_t* data;
	uint16_t length;
}uint16_t_array;

typedef struct uint32_t_array{
	uint32_t* data;
	uint16_t length;
}uint32_t_array;

typedef struct uint64_t_array{
	uint64_t* data;
	uint16_t length;
}uint64_t_array;

typedef struct OBC_cmd{
	int change_power_mode_B16, change_power_mode_B17;
	int doze_B16, reset_B16;
	int doze_B17, reset_B17;
	int change_switch_mode;
	int status_report_B16, status_report_B17;
	int set_switch;
	int set_output_B16, change_output_message_format_B16;
    int set_output_B17, change_output_message_format_B17;
    int read_from_SD_card;
    int change_RTK_function_B17, change_RTK_mode_B17;
    int set_RTK_function_B17, set_RTK_mode_B17;
}OBC_cmd;

typedef struct receiver_status{
	int NMEA_set, binary_set;
	int receiver_on, receiver_dozed;
	int ext_raw_data_set;
}receiver_status;

typedef struct DMAStatus{
	volatile uint8_t stat;
	volatile uint8_t request;
}DMAStatus;
