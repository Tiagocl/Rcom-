// Application layer protocol implementation

#include <string.h>
#include <stdio.h>
#include <sys/stat.h>
#include <stdlib.h>

#include "application_layer.h"
#include "link_layer.h"

//Defines
#define CONTROL_START 1
#define CONTROL_DATA 2
#define CONTROL_END 3
#define CONTROL_PACKET_PARAMETER_FILE_SIZE 0
#define CONTROL_PACKET_PARAMETER_FILENAME 1

int pinguim = FALSE;

////////////////////////////////////////////////
// ApplicationLayer Auxiliar Functions        //
////////////////////////////////////////////////

int controlBuilder(const char *filename,unsigned char *start_packet, unsigned char *end_packet){
    unsigned char temp_packet[MAX_PAYLOAD_SIZE] = {0};
    //unsigned char file_size_hex[20];
    char file_size_hex[20];
    int file_size_bytes = 0;
    long file_size = 0;
    
    if (filename == NULL){
        printf("Error: Filename is NULL\n");
        return -1;
    }
    if (strlen(filename) > 255){
        printf("Error: Filename is too long\n");
        return -1;
    }

    struct stat file_info;
    if (stat(filename, &file_info) != 0){
        printf("Error: Retrieving data from file\n");
    }
    
    sprintf(file_size_hex, "%02lX", file_info.st_size);//If file.st_size is 255, the formatted string stored in hex_string would be "FF".

	//sprintf((char *)file_size_hex, "%02X", (unsigned long) file_info.st_size);
    
    file_size_bytes = strlen(file_size_hex) / 2;
    file_size = file_info.st_size;

    if (file_size_bytes > 256){
        printf("Error: File size is too big\n");
    }
    
    int idx = 0;
    temp_packet[idx++] = CONTROL_START;
    temp_packet[idx++] = CONTROL_PACKET_PARAMETER_FILE_SIZE;
    temp_packet[idx++] = file_size_bytes;          //file size in hex or int?
    
    for (int i = file_size_bytes - 1; i >= 0; i--){
        temp_packet[idx++] = file_size >> (i * 8);
    }

    temp_packet[idx++] = CONTROL_PACKET_PARAMETER_FILENAME;
    temp_packet[idx++] = strlen(filename);

    for (int i = 0; i < strlen(filename); i++){
        temp_packet[idx++] = filename[i]; 
    }

    
    // Assuming packet_size is the size of the array
    memcpy(start_packet, temp_packet, idx * sizeof(unsigned char));
    memcpy(end_packet, temp_packet, idx * sizeof(unsigned char));
    end_packet[0] = CONTROL_END;

    return idx;
}

int dataBuilder(unsigned char* packet, unsigned char* packet_payload, int nSequence, int nBytes) {
    int l2 = nBytes / 256;
    int l1 = nBytes % 256;

    packet[0] = CONTROL_DATA;
    packet[1] = (nSequence + 1) % 100;
    packet[2] = l2;
    packet[3] = l1;

    for (int i = 0; i < nBytes; i++) {
        packet[i + 4] = packet_payload[i];
    }

    return nBytes + 4; // Total size of the data packet
}


////////////////////////////////////////////////
// ApplicationLayer                           //
////////////////////////////////////////////////
void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{
    // TODO
    unsigned char start_packet[MAX_PAYLOAD_SIZE];
    unsigned char end_packet[MAX_PAYLOAD_SIZE];
    int control_packet_size;
    int EOF_flag = 0;
    int data_link_id;
    int packet_size;

    LinkLayer connectionParameters;
    strcpy(connectionParameters.serialPort, serialPort);
    connectionParameters.role = strcmp(role, "tx") == 0 ? LlTx : LlRx;
    connectionParameters.baudRate = baudRate;
    connectionParameters.nRetransmissions = nTries;
    connectionParameters.timeout = timeout;

    data_link_id = llopen(connectionParameters);

    if(data_link_id < 0){
        printf("Error: llopen\n");
        printf("Error: Data Link Id = %d \n", data_link_id);
        return;
    }

    FILE *file;
    if (connectionParameters.role == LlTx){
        // Send file
        file = fopen(filename, "r"); //rb?
        if (file == NULL){
            printf("Error: Opening file\n");
            return;
        }

        // I KEEP this PACKET_SIZE for the next steps SO THAT I CAN BUILD THE DATA PACKET THE PACKET PAYLOAD
        packet_size = controlBuilder(filename, start_packet, end_packet);
        control_packet_size = packet_size;
        printf("//////////////////////////////////////////////////////////////////////");
/*        
        for (int i = 0; i < packet_size; i++){
            printf("0x%02X ", start_packet[i]);
        }
        printf("\n");
        for (int i = 0; i < packet_size; i++){
            printf("0x%02X ", end_packet[i]);
        }
        printf("\n");
*/        
        //printf("APP Packet Size: %d\n", packet_size);
        //unsigned char packet_payload[packet_size - 1]; //char or unsigned char? 
        unsigned char *packet_payload = (unsigned char *)malloc(packet_size); //retirei packet_size - 1 e pus packet_size

        for (int i = 0; i < packet_size; i++){
            packet_payload[i] = start_packet[i];
        }
/*
        printf("Packet Payload:\n");
        for (int i = 0; i < packet_size; i++){
            printf("0x%02X ", packet_payload[i]);
        }
*/        
        printf("____________________________________________________\n");
        int num_written_chars = llwrite(packet_payload, packet_size);
        printf("START APP CONTROL PACKET SENT TO LL: %d\n", packet_size);
        printf("Size of frame sent by LLWRite: %d\n", num_written_chars);

        
        if (num_written_chars < packet_size){
            printf("Error: llwrite\n");
            return;
        }

        free(packet_payload);

        //int cotrol_packet_size = packet_size;



        /*
                if (feof(file)){
                    EOF_flag = 1;
                } else {
                    printf("Error: Reading file\n");
                    return;
                }
                */

        //Até aqui is correct
        packet_payload = (unsigned char *)malloc(MAX_PAYLOAD_SIZE);

        unsigned char data_packet[MAX_PAYLOAD_SIZE] = {0};


        int packet_seq = 0;
        int idx = 0;
        int number_of_data_bytes = 0;
        while (!EOF_flag){
            
            unsigned char read_buffer;
            //int read_buffer;
            if(!fread(&read_buffer, (size_t) 1, (size_t) 1, file)){
                EOF_flag = 1;
                packet_size = dataBuilder(data_packet, packet_payload, packet_seq++, idx); //Removed &packet_payload
                
                printf("EOF\n");
                for(int i = 0; i < packet_size; i++){
                    printf("0x%02X ", data_packet[i]);
                }
                printf("\n");


                num_written_chars = llwrite(data_packet, packet_size);
                if (num_written_chars < 0){
                    printf("Error: llwrite\n");
                    return;
                }

                printf("Size of packet sent by transmiter: %d\n", packet_size);
                printf("Size of frame returned by llwrite: %d\n", num_written_chars);
                       
            } else if((MAX_PAYLOAD_SIZE - 4) == idx){ //MAXPAYLOAD??
                packet_size = dataBuilder(data_packet, packet_payload, packet_seq++, idx);  //Removed &packet_payload
                printf("MAX PAYLOAD SIZE REACHED\n");
                
                //if (MAX_PAYLOAD_SIZE == packet_size) printf("Packet Size == MAX_PAYLOAD_SIZE: %d\n", packet_size);
                
                num_written_chars = llwrite(data_packet, MAX_PAYLOAD_SIZE); //Removed packet_size

                if (num_written_chars < 0){    //Removed packet_size
                    printf("Error: llwrite\n");
                    return;
                }

                printf("Size of packet sent by transmiter: %d\n", packet_size);
                printf("Size of frame returned by llwrite: %d\n", num_written_chars);


                memset(data_packet, 0, sizeof(data_packet));
                memset(packet_payload, 0, sizeof(packet_payload));
                //memset(packet_payload, 0, packet_size - 1);
                idx = 0;
            }

            packet_payload[idx++] = read_buffer;
        }

        fclose(file);
        free(packet_payload);

        //int end_packet_size = sizeof(end_packet) / sizeof(end_packet[0]);
        //memcpy(&packet_payload[idx], end_packet, end_packet_size);

        packet_payload = (unsigned char *)malloc(control_packet_size);
        for (int i = 0; i < control_packet_size; i++){
            packet_payload[i] = end_packet[i];
        }

        num_written_chars = llwrite(packet_payload, control_packet_size);
        if(num_written_chars < 0){
            printf("Error: llwrite\n");
            return;
        }

        printf("START APP CONTROL PACKET SENT TO LL: %d\n", control_packet_size);
        printf("Size of frame sent by LLWRite: %d\n", num_written_chars);
        
        free(packet_payload);
        unsigned char tempbuf[1] = {0};
        
        //printf("Waiting for DISC\n");
        //printf("llclose computed\n");

    } else {
        // Receive file
        FILE *file;
        int READING_flag = 1;

        while (READING_flag){
            unsigned char received_packet[MAX_PAYLOAD_SIZE] = {0};
            int packet_size;
            //int idx = 0;

            printf("Reading\n");
            packet_size = llread(received_packet);
            printf("Packet Size On receiver end: %d\n", packet_size);
            
            
            //printf("Received Packet: ");
            printf("\n");


            if(packet_size == 0){
                READING_flag = 0;
                //could put llwrite here but thats not correct
                break;
            } else{
                if (received_packet[0] == CONTROL_END) {
                    printf("\nClosed penguin\n");
                    pinguim = TRUE;
                    if (file != NULL) {
                        fclose(file);
                    }
                    READING_flag = 0;
                } else if (received_packet[0] == CONTROL_START) {
                    printf("\nOpened penguin\n");
                    file = fopen(filename, "wb");
                    if (file == NULL) {
                        printf("Error: Opening file\n");
                        return;
                    }
                } else{
                    for (int i = 4; i < packet_size; i++) {
                        fputc(received_packet[i], file);
                    }
                }
            }
        }
    }
    // NEEd to put something ere to send disc frame
    llclose(data_link_id);
    
}
