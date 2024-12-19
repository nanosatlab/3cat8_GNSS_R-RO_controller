/*
 * nmea.h
 *
 * Description: Header file for handling NMEA sentence parsing and processing.
 *              This file contains the function declarations for parsing
 *              and extracting information from NMEA 0183 GPS data sentences.
 *
 * Author: Jordi González de Regàs
 * Date: 2024
 *
 * Functions:
 *    - int Checksum_NMEA(char* message);
 *    - char* Get_Talker_ID(char* message);
 *    - char* Get_Mode(char* message);
 *    - char* Get_UTCTime(char* message, char* mode);
 *    - char* Get_Latitude(char* message, char* mode);
 *    - char* Get_Longitude(char* message, char* mode);
 *    - char* Get_Altitude(char* message, char* mode);
 *    - char* Get_UTCDate(char* message, char* mode);
 *    - int is_NMEA(char* message);
 *    - void Process_NMEA_Data(char* buffer_str_cpy);
 *
 * Notes:
 *    - The functions declared here are implemented in nmea.c
 *    - Ensure the proper inclusion of required headers in the implementation file.
 *
 */
#ifndef NMEA_H
#define NMEA_H

// Declare the functions
int Checksum_NMEA(char* message);
char* Get_Talker_ID(char* message);
char* Get_Mode(char* message);
char* Get_UTCTime(char* message, char* mode);
char* Get_Latitude(char* message, char* mode);
char* Get_Longitude(char* message, char* mode);
char* Get_Altitude(char* message, char* mode);
char* Get_UTCDate(char* message, char* mode);
int is_NMEA(char* message);
void Process_NMEA_Data(char* buffer_str_cpy);

#endif // NMEA_H
