/*file name: nmea.c*/

#include "nmea.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "types.h"
#include "fatfs.h"
#include "main.h"

int Checksum_NMEA(char* message){

/*
Computes the Checksum according to the NMEA convention: perform bitwise XOR of the ASCII representation of the message from $ to *
both not included. Ending of the message is * checksum_1 checksum_2 <LF><CR>, so i<len(message)-5.
input: char* message, which is a string containing the message eligible for computing checksum.
APPROVED FOR FLIGHT
*/

    int checksum = 0;

    // Find the start of the NMEA message (after '$') and before '*'
    if (message[0] == '$') {
        message++;  // Skip the '$' symbol
    }

    while (*message != '*' && *message != '\0') {
        checksum ^= *message;  // XOR the character with the checksum
        message++;
    }

return checksum;
}

char* Get_Talker_ID(char* message){
/*Returns the Talker ID from a NMEA message:
* GP: GPS
* GL: GLONASS
* GA: GALILEO
* BD or GB: BEIDOU
* input: *char NMEA message as a string.
* returns: char* string containing ID code.
* */
char* id = (char*)malloc(3*sizeof(char));
id[0] = message[1];
id[1] = message[2];
id[2] = '\0';

return id;
}
char* Get_Mode(char* message){
/*Returns the mode from a NMEA message:
* GGA: GPS Fixed Data
* GLL: Geographic Position, Latitude and Longitude
* GSA: GNSS DOP & Active Satellites
* GSV: GNSS Satellites in View.
* RMC: Recommended Minimum Specific GPS Data
* VTG: Course over ground and ground speed.
* input: char* NMEA message as a string.
* returns: char* string containing mode code.
* */
char* mode = (char*)malloc(4*sizeof(char));
mode[0] = message[3];
mode[1] = message[4];
mode[2] = message[5];
mode[3] = '\0';
return mode;
}

char* Get_UTCTime(char* message, char* mode){
    /*Gets UTC Time from NMEA message (hhmmss.ssss).
    inputs: NMEA message in the form of char*, mode of the message in char* (run Get_Mode() first).
    outputs: char* containing the UTC Time in a string (raw).*/
    char* result =NULL;

    if((strcmp(mode, "GGA")==0)||(strcmp(mode, "GLL")==0)||(strcmp(mode, "ZDA")==0) || (strcmp("RMC", mode)==0)||(strcmp("GNS", mode)==0)){
        char* message_cpy= (char*)malloc((strlen(message)+1)*sizeof(char));
        int chop_count = 0;
        strcpy(message_cpy, message);
        message_cpy[strlen(message)] = '\0';
        char* original_message_cpy = message_cpy;
        for(char* chunk = strsep(&message_cpy, ","); chunk!=NULL; chunk=strsep(&message_cpy,",")){

            if((strcmp(mode, "GLL")==0) && (chop_count == 5)){

                result = (char*)malloc(strlen(chunk+1)*sizeof(char));
                strcpy(result, chunk);
                result[strlen(chunk)] = '\0';
                break;
            }

            if(((strcmp(mode, "ZDA")==0)||(strcmp(mode, "GGA")==0)||(strcmp(mode, "RMC")==0)||(strcmp(mode, "GNS")==0)) && (chop_count == 1)){

                result = (char*)malloc((strlen(chunk)+1)*sizeof(char));
                strcpy(result, chunk);
                result[strlen(chunk)] = '\0';
                break;
            }


            chop_count++;
        }

        free(original_message_cpy);

    }

    else{
        result = malloc(3*sizeof(char));
        if(result==NULL){exit(-1);}
        result[0] = 'N';
        result[1] = 'A';
        result[2] = '\0';
    }
    return result;
}
char* Get_Latitude(char* message, char* mode){
    /*Gets the latitude from a NMEA message.
    inputs: char* containing NMEA message, char* containing the mode of the message.
    outputs: char* containing the latitude in the form ddmm.mmmmN/S, where N/S is the hemisphere.*/

    char* result = NULL;

    if(strcmp("GLL", mode)==0 || strcmp("GGA", mode)==0 || strcmp("RMC", mode)==0 ||strcmp("GNS", mode)==0 ){
        char* message_cpy= (char*)malloc((strlen(message)+1)*sizeof(char));
        int chop_count = 0;
        strcpy(message_cpy, message);
        message_cpy[strlen(message)]='\0';
        char* original_message_cpy = message_cpy;
        for(char* chunk = strsep(&message_cpy, ","); chunk!=NULL; chunk=strsep(&message_cpy,",")){


            if((strcmp("GLL", mode)==0) && (chop_count == 1)){
                result = (char*)malloc((strlen(chunk)+2)*sizeof(char));
                strcpy(result, chunk);
                chunk = strsep(&message_cpy, ",");
                strcat(result, chunk);
                result[strlen(result)]='\0';

                break;
            }

            if(((strcmp("GGA", mode)==0) || (strcmp("GNS", mode)==0)) && (chop_count == 2)){
                result = (char*)malloc((strlen(chunk)+2)*sizeof(char));
                strcpy(result, chunk);
                chunk = strsep(&message_cpy, ",");
                strcat(result, chunk);
                result[strlen(result)]='\0';
                break;
            }

            if((strcmp("RMC", mode)==0) && (chop_count == 3)){
                result = (char*)malloc((strlen(chunk)+2)*sizeof(char));
                strcpy(result, chunk);
                chunk = strsep(&message_cpy, ",");
                strcat(result, chunk);
                result[strlen(result)]='\0';
                break;
            }
            chop_count++;
        }

        free(original_message_cpy);

    }
    else{
        result = malloc(3*sizeof(char));
        if(result==NULL){exit(-1);}
        result[0] = 'N';
        result[1] = 'A';
        result[2] = '\0';
    }

    return result;
}
char* Get_Longitude(char* message, char* mode){
    /*Gets the longitude from a NMEA message.
    inputs: char* containing NMEA message, char* containing the mode of the message.
    outputs: char* containing the latitude in the form dddmm.mmmmE/W, where E/W is the hemisphere.*/

    char* result = NULL;

    int is_GLL = (strcmp("GLL", mode)==0);
    int is_GGA = (strcmp("GGA", mode)==0);
    int is_RMC = (strcmp("RMC", mode)==0);
    int is_GNS = (strcmp("GNS", mode)==0);

    if(is_GLL||is_GGA||is_RMC||is_GNS){
        char* message_cpy= (char*)malloc((strlen(message)+1)*sizeof(char));
        int chop_count = 0;
        strcpy(message_cpy, message);
        message_cpy[strlen(message)]='\0';
        char* original_message_cpy = message_cpy;
        for(char* chunk = strsep(&message_cpy, ","); chunk!=NULL; chunk=strsep(&message_cpy,",")){


            if(is_GLL && (chop_count == 3)){
                result = (char*)malloc((strlen(chunk)+2)*sizeof(char));
                strcpy(result, chunk);
                chunk = strsep(&message_cpy, ",");
                strcat(result, chunk);
                result[strlen(result)] = '\0';
                break;
            }

            if((is_GGA || is_GNS) && (chop_count == 4)){
                result = (char*)malloc((strlen(chunk)+2)*sizeof(char));
                strcpy(result, chunk);
                chunk = strsep(&message_cpy, ",");
                strcat(result, chunk);
                result[strlen(result)] = '\0';
                break;
            }

            if(is_RMC && (chop_count == 5)){
                result = (char*)malloc((strlen(chunk)+2)*sizeof(char));
                strcpy(result, chunk);
                chunk = strsep(&message_cpy, ",");
                strcat(result, chunk);
                result[strlen(result)] = '\0';
                break;
            }
            chop_count++;
        }

        free(original_message_cpy);
    }

    else{
        result = malloc(3*sizeof(char));
        if(result==NULL){exit(-1);}
        result[0] = 'N';
        result[1] = 'A';
        result[2] = '\0';
    }

    return result;
}
char* Get_Altitude(char* message, char* mode){
    /*Gets the MSL altitude [m] from a NMEA message.
    inputs: char* containing NMEA message, char* containing the mode of the message.
    outputs: char* containing the MSL altitude in meters.*/

    char* result = NULL;

    int is_GGA = (strcmp("GGA", mode)==0);
    int is_GNS = (strcmp("GNS", mode)==0);

    if(is_GGA || is_GNS){

        char* message_cpy = (char*)malloc((strlen(message)+1)*sizeof(char));
        int chop_count = 0;
        strcpy(message_cpy, message);
        message_cpy[strlen(message)]='\0';
        char* original_message_cpy = message_cpy;

        for(char* chunk = strsep(&message_cpy, ","); chunk!=NULL; chunk=strsep(&message_cpy,",")){
            if((is_GGA || is_GNS) && (chop_count == 9)){
            result = (char*)malloc((strlen(chunk)+1)*sizeof(char));
            strcpy(result, chunk);
            result[strlen(chunk)] = '\0';
            break;
            }
            chop_count++;
        }

        free(original_message_cpy);

    }

    else{
        result = malloc(3*sizeof(char));
        if(result==NULL){exit(-1);}
        result[0] = 'N';
        result[1] = 'A';
        result[2] = '\0';
    }

    return result;

}
char* Get_UTCDate(char* message, char* mode) {
    /* Gets the Date in UTC from the NMEA messages.
     * Inputs: char* containing NMEA message, char* containing the mode of the message.
     * Outputs: char* containing the date. In ZDA: ddmmyyyy, in RMC: ddmmyy. */

    char* result = NULL;  // Initialize to NULL
    int is_ZDA = (strcmp("ZDA", mode) == 0);
    int is_RMC = (strcmp("RMC", mode) == 0);

    if (is_ZDA || is_RMC) {
        // Duplicate message to avoid modifying the original
        char* message_cpy = strdup(message);
        if (message_cpy == NULL) {
            exit(-1);  // Handle memory allocation failure
        }

        // Save the original pointer for later use in free()
        char* original_message_cpy = message_cpy;

        int chop_count = 0;
        char* chunk = NULL;

        // Tokenize the message by commas
        while ((chunk = strsep(&message_cpy, ",")) != NULL) {
            chop_count++;

            // Handling for ZDA mode (chop_count == 3)
            if (is_ZDA && (chop_count == 3)) {
                result = malloc(9 * sizeof(char));  // Allocate space for ddmmyyyy
                if (result == NULL) {
                    free(original_message_cpy);  // Ensure we free memory before exit
                    exit(-1);
                }
                strcpy(result, chunk);  // Copy the day
                chunk = strsep(&message_cpy, ",");  // Get month
                strcat(result, chunk);
                chunk = strsep(&message_cpy, ",");  // Get year
                strcat(result, chunk);
                result[8] = '\0';
                break;
            }

            // Handling for RMC mode (chop_count == 10)
            if (is_RMC && (chop_count == 10)) {
                result = malloc((strlen(chunk) + 1) * sizeof(char));  // +1 for null terminator
                if (result == NULL) {
                    free(original_message_cpy);  // Ensure we free memory before exit
                    exit(-1);
                }
                strcpy(result, chunk);  // Copy the date string
                result[strlen(chunk)]= '\0';
                break;
            }
        }

        // Free the original message copy after processing
        free(original_message_cpy);
    }

    else{
        result = malloc(3*sizeof(char));
        if(result==NULL){exit(-1);}
        result[0] = 'N';
        result[1] = 'A';
        result[2] = '\0';
    }
    return result;  // Return the date result
}

int is_NMEA(char* message){
    /*Checks whether a message is NMEA (without verifying the checksum)*/
    int is_nmea = 0;

    if((message[0] == '$')&&(strchr(message, '*')!=NULL) && (strlen(message)>=6)){
        char* talker_id = Get_Talker_ID(message);
        char* mode = Get_Mode(message);
        char modes[] = "ZDA RMC GGA VTG GNS GLL GSV GSA";
        char talker_ids[] = "GP GN GA GB BD GL";

        if((strstr(modes, mode)!=NULL) && (strstr(talker_ids, talker_id)!=NULL)){
            is_nmea = 1;
        }

        free(talker_id);
        free(mode);
    }

    return is_nmea;

}


