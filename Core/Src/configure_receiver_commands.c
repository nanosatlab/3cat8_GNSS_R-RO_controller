/*
 *configure_receiver_commands.c
 * */


#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "types.h"
#include "main.h"
#include "configure_receiver_commands.h"

uint8_t_array Receiver_Cold_Restart(uint8_t start_mode, uint8_t UTCday, uint8_t UTCmonth, uint16_t UTCyear, uint8_t UTChour, uint8_t UTCminute, uint8_t UTCsecond, signed int lat, signed int lon, signed int alt){
    /*Builds command for B16/17 to perform a cold restart.
    inputs: UTC day, month, year, hour, minute, second.
    latitude, longitude, altitude.
    All these can be input in decimal directly without issue.
    output: uint8_t_array containing the command. */
    uint8_t_array cmd;
    cmd.length = 22;
    cmd.data = malloc(cmd.length*sizeof(uint8_t));
    if(cmd.data==NULL){exit(-1);}
    cmd.data[0] = 0xA0, cmd.data[1] = 0xA1, cmd.data[2] = 0x00, cmd.data[3] = 0x0F, cmd.data[4] = 0x01;
    cmd.data[5] = start_mode; //Cold start 0x03...
    cmd.data[6] = (UTCyear>>8) & 0xFF, cmd.data[7] = UTCyear & 0xFF;
    cmd.data[8] = UTCmonth;
    cmd.data[9] = UTCday;
    cmd.data[10] = UTChour, cmd.data[11] = UTCminute, cmd.data[12] = UTCsecond;
    cmd.data[13] = (lat >> 8) & 0xFF, cmd.data[14] = lat & 0xFF;
    cmd.data[15] = (lon >> 8) & 0xFF, cmd.data[16] = lon & 0xFF;
    cmd.data[17] = (alt >> 8) & 0xFF, cmd.data[18] = alt & 0xFF;

    for(int i = 4; i<19; i++){
        cmd.data[19] ^= cmd.data[i];
    }

    cmd.data[20] = 0x0D;
    cmd.data[21] = 0x0A;
    return cmd;
}

uint8_t_array Configure_RTK_Mode_and_Operational_Function(uint8_t mode, uint8_t function, uint32_t survey_length, uint32_t standard_deviation, double latitude, double longitude, float altitude, float baseline_length_constraint, uint8_t attributes){
	uint8_t_array cmd;
	uint8_t checksum = 0;
	uint32_t survey_length_cpy = 0;
	uint32_t standard_deviation_cpy = 0;
	cmd.length = 7+37;
	cmd.data = malloc(cmd.length*sizeof(uint8_t));
	cmd.data[0] = 0xA0;
	cmd.data[1] = 0xA1;
	cmd.data[2] = 0x00;
	cmd.data[3] = 0x25;
	cmd.data[4] = 0x6A;
	cmd.data[5] = 0x06;
	cmd.data[6] = mode;
	cmd.data[7] = function;

	if(mode != RTK_BASE){

		for(int i=8; i<cmd.length-4;i++){
			cmd.data[i] = 0x00;
		}
		cmd.data[cmd.length-4] = attributes;
	}

	else{

		if((survey_length >= 60) && (survey_length <= 1209600)){
			survey_length_cpy = survey_length;
		}else{survey_length_cpy = 60;}

		if((standard_deviation >= 3) && (standard_deviation <= 100)){
			standard_deviation_cpy = standard_deviation;
		}else{standard_deviation_cpy = 3;}

		for(int i=0; i<4; i++){
			cmd.data[i+8] = (survey_length_cpy>>(8*(3-i)) ) & 0xFF;
			cmd.data[i+12] = (standard_deviation_cpy>>(8*(3-i)) ) & 0xFF;
		}

		uint8_t* lat_ptr = (uint8_t*) &latitude;
		uint8_t* lon_ptr = (uint8_t*) &longitude;
		uint8_t* alt_ptr = (uint8_t*) &altitude;
		uint8_t* blc_ptr = (uint8_t*) &baseline_length_constraint;
		//Has to be fed into the receiver as big endian!!!! Check when testing...
		for(int i=0; i<8; i++){
			cmd.data[i+16] = lat_ptr[7-i];
		    cmd.data[i+24] = lon_ptr[7-i];
		}
		for(int i=0; i<4; i++){
			cmd.data[i+32] = alt_ptr[7-i];
			cmd.data[i+36] = blc_ptr[7-i];
		}
	}

    for(int i=4; i<cmd.length-3; i++){
        checksum ^= cmd.data[i];
    }

    cmd.data[cmd.length-3] = checksum;
    cmd.data[cmd.length-2] = 0x0D;
    cmd.data[cmd.length-1] = 0x0A;

	return cmd;
}

uint8_t_array Configure_Binary_Measurement_Data_Output(uint8_t frequency, uint8_t MeasTimeEnabling, uint8_t RawMeasEnabling, uint8_t SV_CH_StatusEnabling, uint8_t RCVEnabling, uint8_t SubframeDifferentCV, uint8_t ExtendedRawMeas, uint8_t attr){
    /*Builds the command to configure binary measurement data output on B16/17. Takes the frequency in form of int. */
    uint8_t_array command;
    command.length = 16;
    command.data = (uint8_t*)malloc(command.length*sizeof(uint8_t));
    uint8_t checksum = 0x00;
    int i = 0;
    //prologue of command message (payload length 9).
    command.data[0] = 0xA0;
    command.data[1] = 0xA1;
    command.data[2] = 0x00;
    command.data[3] = 0x09;
    command.data[4] = 0x1E;
    switch(frequency){
        case 1:
            command.data[5] = 0x00;
            break;
        case 2:
            command.data[5] = 0x01;
            break;
        case 4:
            command.data[5] = 0x02;
            break;
        case 5:
            command.data[5] = 0x03;
            break;
        case 10:
            command.data[5] = 0x04;
            break;
        case 20:
            command.data[5] = 0x05;
            break;
        case 8:
            command.data[5] = 0x06;
            break;
        default:
            command.data[5] =0xFF;
    }

    command.data[6] = MeasTimeEnabling;
    command.data[7] = RawMeasEnabling;
    command.data[8] = SV_CH_StatusEnabling;
    command.data[9] = RCVEnabling;
    command.data[10] = SubframeDifferentCV;
    command.data[11] = ExtendedRawMeas;
    command.data[12] = attr;

    for(i=4; i<command.length-3; i++){
        checksum ^= command.data[i];
    }
    //ending of command message
    command.data[command.length-3] = checksum;
    command.data[command.length-2] = 0x0D;
    command.data[command.length-1] = 0x0A;

    return command;
}

uint8_t_array Build_ACK(uint8_t_array msgID){
	/*Builds an ACK message given a msgID hex ID code of the order sent to the GNSS receiver (check manual for specific values).
	 * inputs: hex with the ID code of the order sent to the GNSS receiver.
	 * outputs: uint8_t_array of hex values containing the ACK message.*/
	uint8_t_array cmd;
	cmd.length = 8+msgID.length;
	cmd.data = (uint8_t*)malloc(cmd.length*sizeof(uint8_t));
	if(cmd.data==NULL){exit(-1);}
	uint8_t checksum = 0x00;
	int i = 0;

	cmd.data[0] = 0xA0;
	cmd.data[1] = 0xA1;
	cmd.data[2] = ((msgID.length+0x01) >> 8) & 0xFF;
	cmd.data[3] = (msgID.length+0x01) & 0xFF;
	cmd.data[4] = 0x83;
	for(i=0; i<msgID.length; i++){
		cmd.data[i+5] = msgID.data[i];
	}
	for(i=4; i<=4+msgID.length; i++){checksum^=cmd.data[i];}
	cmd.data[5+msgID.length] = checksum;
	cmd.data[6+msgID.length] = 0x0D;
	cmd.data[7+msgID.length] = 0x0A;

	return cmd;
}

uint8_t_array Build_NACK(uint8_t_array msgID){
	/*Builds a NACK message given a msgID hex ID code of the order sent to the GNSS receiver (check manual for specific values).
	 * inputs: hex with the ID code of the order sent to the GNSS receiver.
	 * outputs: uint8_t_array of hex values containing the NACK message.*/
	uint8_t_array cmd;
	cmd.length = 8+msgID.length;
	cmd.data = (uint8_t*)malloc(cmd.length*sizeof(uint8_t));
	if(cmd.data==NULL){exit(-1);}
	uint8_t checksum = 0x00;
	int i = 0;

	cmd.data[0] = 0xA0;
	cmd.data[1] = 0xA1;
	cmd.data[2] = ((msgID.length+0x01) >> 8) & 0xFF;
	cmd.data[3] = (msgID.length+0x01) & 0xFF;
	cmd.data[4] = 0x84;
	for(i=0; i<msgID.length; i++){
		cmd.data[i+5] = msgID.data[i];
	}
	printf("%X \n", cmd.data[5]);
	for(i=4; i<=4+msgID.length; i++){checksum^=cmd.data[i];}
	cmd.data[5+msgID.length] = checksum;
	cmd.data[6+msgID.length] = 0x0D;
	cmd.data[7+msgID.length] = 0x0A;

	return cmd;
}

uint8_t_array Configure_NMEA_Message_Interval(uint8_t* intervals){
	/*Builds a Configure NMEA message interval to modify the time intervals
	 * input: uint8_t* of values 0x00 to 0xFF that contains the times in seconds of the intervals of each message type and the attributes at the last bit
	 * must be of length 13: {GGA, GSA, GSV, GLL, RMC, VTG, ZDA, GNS, GBS, GRS, DTM, GST, ATTRIBUTES}
	 * ATTRIBUTES: 00 update to SRAM. 01 update to SRAM & FLASH.*/
	uint8_t_array command;
	int i = 0;
	uint8_t checksum = 0x00;
	command.length = 22;
	command.data = (uint8_t*)malloc(command.length*sizeof(uint8_t));
	command.data[0] = 0xA0;
	command.data[1] = 0xA1;
	command.data[2] = 0x00;
	command.data[3] = 0x0F;
	command.data[4] = 0x64;
	command.data[5] = 0x02;
	for(i=6; i<19; i++){
		command.data[i] =  intervals[i-6];
	}
	for(i=4; i<19; i++){
		checksum ^= command.data[i];
	}
	command.data[19] = checksum;
	command.data[20] = 0x0D;
	command.data[21] = 0x0A;

	return command;
}

uint8_t_array Configure_Navigation_Data_Message_Interval(uint8_t interval, uint8_t attr){
	/*Builds a message to configure the navigation data (0x8A) message interval in seconds.
	 * inputs: interval, (0-255) in seconds. attr: UPDATE_TO_SRAM or UPDATE_TO_SRAM_AND_FLASH*/
	uint8_t_array cmd;
	cmd.length = 11;
	cmd.data = malloc(cmd.length*sizeof(uint8_t));
	if(cmd.data == NULL){exit(-1);}
	cmd.data[0] = 0xA0, cmd.data[1] = 0xA1, cmd.data[cmd.length-2] = 0x0D, cmd.data[cmd.length-1] = 0x0A;
	cmd.data[2] = 0x00, cmd.data[3] = 0x04, cmd.data[4] = 0x64, cmd.data[5] = 0x2F;
	cmd.data[6] = interval, cmd.data[7] = attr;
	cmd.data[8] = 0x00;
	for(int i = 4; i<cmd.length-3; i++){
		cmd.data[8] ^= cmd.data[i];
	}

	return cmd;
}
uint8_t_array Change_Message_Format(char* message_type, uint8_t update_memory){
	/*--EXPERIMENTAL--


	 * Builds the command message in hex to change the message format.
	 * inputs: message_type string which can be either "NMEA", "Binary" or "No" depending on if we want the messages to be sent in NMEA, binary, or no message to be sent, respectively.
	 * uint8_t containing update_memory, with options UPDATE_TO_SRAM or UPDATE_TO_SRAM_AND_FLASH depending on if we want the order to be stored only in SRAM or also in FLASH.
	 * Check Phoenix receiver (skytraq) manual for more information.
	 * */
	uint8_t_array command;
	command.length = 10;
	command.data = (uint8_t*)malloc(command.length*sizeof(uint8_t));
	if(command.data==NULL){exit(-1);}
	uint8_t checksum = 0x00;
	int i = 0;

	command.data[0] = 0xA0;
	command.data[1] = 0xA1;
	command.data[2] = 0x00;
	command.data[3] = 0x03; //2-3: payload length is always 3 for this message.
	command.data[4] = 0x09;
	if(strcmp(message_type, "NMEA")==0){
		command.data[5] = 0x01;
	}
	if(strcmp(message_type, "Binary")==0){
		command.data[5] = 0x02;
	}
	if(strcmp(message_type, "No")==0){
		command.data[5] = 0x00;
	}
	command.data[6] = update_memory;

	for(i=4; i<7; i++){ //Checksum starts from msg ID to end of payload
		checksum ^= command.data[i];
	}
	command.data[7] = checksum;
	command.data[8] = 0x0D;
	command.data[9] = 0x0A;

	return command;
}

uint8_t_array Build_uint8_t_Command(uint8_t* array, uint16_t size){
	uint8_t_array cmd;
	cmd.length = size;
	cmd.data = malloc(cmd.length*sizeof(uint8_t));
	if(cmd.data == NULL){exit(-1);}
	cmd.data = array;
	return cmd;
}
