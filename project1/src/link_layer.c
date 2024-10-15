// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source

//Link layer specific values
#define FLAG 0x7E
#define SENDER_ADDRESS 0x03
#define RECEIVER_ADDRESS 0x01

// Supervision and Unnumbered Frame Specific Values
#define CONTROL_SET 0x03
#define CONTROL_UA 0x07
#define CONTROL_RR_0 0xAA
#define CONTROL_RR_1 0xAB
#define CONTROL_REJ_0 0x54
#define CONTROL_REJ_1 0x55
#define CONTROL_DISC 0x0B

//Information Frame Specific Values
#define INFO_FRAME_0 0x00
#define INFO_FRAME_1 0x80





////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters)
{
    if (openSerialPort(connectionParameters.serialPort,
                       connectionParameters.baudRate) < 0)
    {
        return -1;
    }

    // TODO

    return 1;
}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize)
{
    // TODO

    return 0;
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet)
{
    // Loop for input
    unsigned char buf[BUF_SIZE + 1] = {0}; // +1: Save space for the final '\0' char

    while (STOP == FALSE)
    {
        // Returns after 5 chars have been input
        int bytes = read(fd, buf, BUF_SIZE);
        buf[bytes] = '\0'; // Set end of string to '\0', so we can printf

        for (int i = 0; i < bytes; i++){
            printf("var = 0x%02X\n", buf[i]); // Print the variable var in hexadecimal format
        }
        
        // Check if the received frame is an set frame
        if(state_machine(buf) != 0){
            printf("State machine error\n");
            continue;
        }else{
            buf[0] = FLAG;
            buf[1] = RECEIVER_ADDRESS;
            buf[2] = CONTROL_UA;
            buf[3] = RECEIVER_ADDRESS ^ CONTROL_UA;
            buf[4] = FLAG;
            buf[5] = '\0';
            int response = write(fd, buf, BUF_SIZE);
            break;
        }


        printf(":%s:%d\n", buf, bytes);
        
    }

    return 0;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose(int showStatistics)
{
    // TODO

    int clstat = closeSerialPort();
    return clstat;
}
