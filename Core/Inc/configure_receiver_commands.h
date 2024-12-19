/*
 * configure_receiver_commands.h
 *
 * Description: Header file for handling the craft of commands to be sent to skytraq B16 and B17 GNSS receivers when configuring it.
 *
 * Author: Jordi González de Regàs
 * Date: 2024
 *
 * Functions:
 *
 * Notes:
 *    - The functions declared here are implemented in configure_receiver_commands.c
 *
 */


#ifndef CONFIGURE_RECEIVER_COMMANDS_H
#define CONFIGURE_RECEIVER_COMMANDS_H

//RTK modes
#define RTK_ROVER						0x00
#define RTK_BASE						0x01
#define	RTK_PRECISELY_KINEMATIC_BASE	0x02

//RTK operational functions:
#define RTK_ROVER_NORMAL				0x00
#define RTK_ROVER_FLOAT					0x01
#define RTK_ROVER_MOVING_BASE			0x02
#define RTK_BASE_KINEMATIC				0x00
#define	RTK_BASE_SURVEY					0x01
#define RTK_BASE_STATIC					0x02
#define RTK_PRECISELY_KINEMATIC_NORMAL	0x00
#define RTK_PRECISELY_KINEMATIC_FLOAT	0x01

// Declare the functions
uint8_t_array Receiver_Cold_Restart(uint8_t, uint8_t, uint8_t, uint16_t, uint8_t, uint8_t, uint8_t, signed int, signed int, signed int);

uint8_t_array Configure_Binary_Measurement_Data_Output(uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t);

uint8_t_array Build_ACK(uint8_t_array);

uint8_t_array Build_NACK(uint8_t_array);

uint8_t_array Configure_NMEA_Message_Interval(uint8_t*);

uint8_t_array Configure_Navigation_Data_Message_Interval(uint8_t, uint8_t);

uint8_t_array Change_Message_Format(char*, uint8_t);

uint8_t_array Build_uint8_t_Command(uint8_t*, uint16_t);

uint8_t_array Configure_RTK_Mode_and_Operational_Function(uint8_t, uint8_t, uint32_t, uint32_t, double, double, float, float, uint8_t);

#endif // CONFIGURE_RECEIVER_COMMANDS_H
