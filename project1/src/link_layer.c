// Link layer protocol implementation

#include <fcntl.h> // File control works but when I include it, it gives me an error
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <termios.h>
#include <math.h>
#include <stdio.h>
#include <time.h>
#include <stdbool.h>
#include <unistd.h>

// #include <signal.h> //enabling functionality of alarm

#include "link_layer.h"
#include "serial_port.h"
#include "alarm.h"

int fd; // Global File descriptor

// Global Variables

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source
#define BUF_SIZE 256

// Link layer specific values
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

// Information Frame Specific Values
#define INFO_FRAME_0 0x00
#define INFO_FRAME_1 0x80

typedef enum states (*State_transition)(void);

enum states
{
    STATE_START,
    STATE_FLAG_RCV,
    STATE_A_RCV,
    STATE_C_RCV,
    STATE_BCC_OK,
    STATE_STOP
};

enum events
{
    FLAG_EVENT = FLAG,
    SENDER_ADDRESS_EVENT = SENDER_ADDRESS,
    RECEIVER_ADDRESS_EVENT = RECEIVER_ADDRESS,
    CONTROL_SET_EVENT = CONTROL_SET,
    CONTROL_UA_EVENT = CONTROL_UA,
    CONTROL_DISC_EVENT = CONTROL_DISC,
    CONTROL_RR_0_EVENT = CONTROL_RR_0,
    CONTROL_RR_1_EVENT = CONTROL_RR_1,
    CONTROL_REJ_0_EVENT = CONTROL_REJ_0,
    CONTROL_REJ_1_EVENT = CONTROL_REJ_1,
    BCC1_EVENT, // need to assign after calculating
    BCC2_EVENT  // need to assign after calculating
};

typedef struct
{
    enum states origin;
    enum events input;
    State_transition transition_function;
} transition;

static enum states goto_START(void) { return STATE_START; };
static enum states goto_FLAG_RCV(void) { return STATE_FLAG_RCV; };
static enum states goto_A_RCV(void) { return STATE_A_RCV; };
static enum states goto_C_RCV(void) { return STATE_C_RCV; };
static enum states goto_BCC_OK(void) { return STATE_BCC_OK; };
static enum states goto_STOP(void) { return STATE_STOP; };

transition transitions[] = {
    {STATE_START, FLAG_EVENT, goto_FLAG_RCV},
    {STATE_FLAG_RCV, FLAG_EVENT, goto_FLAG_RCV},
    {STATE_FLAG_RCV, SENDER_ADDRESS_EVENT, goto_A_RCV},
    {STATE_A_RCV, FLAG_EVENT, goto_FLAG_RCV},
    {STATE_A_RCV, CONTROL_SET_EVENT, goto_C_RCV},
    {STATE_A_RCV, CONTROL_UA_EVENT, goto_C_RCV},
    {STATE_A_RCV, CONTROL_DISC_EVENT, goto_C_RCV},
    {STATE_A_RCV, CONTROL_RR_0_EVENT, goto_C_RCV},
    {STATE_A_RCV, CONTROL_RR_1_EVENT, goto_C_RCV},
    {STATE_A_RCV, CONTROL_REJ_0_EVENT, goto_C_RCV},
    {STATE_A_RCV, CONTROL_REJ_1_EVENT, goto_C_RCV},
    {STATE_C_RCV, BCC1_EVENT, goto_BCC_OK},
    {STATE_BCC_OK, FLAG_EVENT, goto_STOP},
    //{STATE_STOP, BCC2, goto_START} //This is the last state, so it should go back to the start?

};
#define TRANS_COUNT sizeof(transitions) / sizeof(transitions[0])

// Notes:
/*
Header is composed of Address, Control Field, BCC
Rx State Machine receives a Frame

*/
// requirements of the state machine:

/*
- A frame can be started with one or more flags___________________done
- Frames with wrong header are ignored ___________________________done
- I Frames without errors in the header and data are accepted
    - New frame: data field is accepted and confirmed with RR
    - Duplicate: data field is discarded and confirmed with RR
- I Frames with no error in the header but error in the data field (BCC)- Data field is discarded but control field is used to trigger action:
    - New Frame: make retransmission request with REJ to anticipate time-out from transmitter
    - Duplicate: Confirmed with RR ???
- I, Set, Disc frames are protected by a timer
    - In the event of a time-out a max number of retransmissions must be made

-
*/

// The state machine
int state_machine(unsigned char buf[], int readBytes)
{

    enum states current_state = STATE_START; /////////////////////////////////////////////////////

    for (int i = 0; i < readBytes && current_state != STATE_STOP; i++)
    {
        bool OTHER_RCV = TRUE; // If it's none of the defined transitions (OTHER_RCV), goto_START

        for (int j = 0; j < TRANS_COUNT; j++)
        {
            if (transitions[j].origin == current_state)
            {
                if (current_state == STATE_C_RCV)
                {
                    if (i >= 2 && buf[i] == (buf[i - 1] ^ buf[i - 2]))
                    { // BCC1 = A ^ C
                        // BCC1_EVENT = buf[i];///ERROR TRYNG
                        current_state = goto_BCC_OK();
                        OTHER_RCV = FALSE;
                    }
                }
                if (transitions[j].input == buf[i])
                {
                    current_state = transitions[j].transition_function();
                    OTHER_RCV = FALSE;
                    break;
                }
            }
        }

        if (OTHER_RCV)
        {
            if (buf[i] == FLAG)
            {
                current_state = goto_FLAG_RCV();
                continue;
            }
            else
            {
                current_state = goto_START();
                continue;
            }
        }
    }

    if (current_state == STATE_STOP)
        return TRUE;

    return FALSE;
}
////////////////////////////////////////////////
// LLOPEN - AUXILIARY FUNCTIONS
////////////////////////////////////////////////

int checkUAFrame(int fd, unsigned char *receiverBuf, size_t bufSize)
{
    int response = read(fd, receiverBuf, bufSize);
    if (response < 0)
    {
        perror("Error receiving UA frame");
        return -1;
    }

    if (response < 5)
    {
        printf("Error: Incomplete UA frame received\n");
        return -1;
    }

    if (receiverBuf[0] != FLAG || receiverBuf[4] != FLAG)
    {
        printf("Error: UA frame FLAG field mismatch\n");
        return -1;
    }

    if (receiverBuf[2] != CONTROL_UA)
    {
        printf("Error: UA frame CONTROL field mismatch\n");
        return -1;
    }

    unsigned char bcc1 = receiverBuf[1] ^ receiverBuf[2];
    if (receiverBuf[3] != bcc1)
    {
        printf("Error: UA frame BCC1 field mismatch\n");
        return -1;
    }

    return 0;
}

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters)
{
    // Open serial port device for reading and writing and not as controlling tty
    // because we don't want to get killed if linenoise sends CTRL-C.

    // NONBLOCKING IS MISSIng
    fd = open(connectionParameters.serialPort, O_RDWR | O_NOCTTY);
    if (fd < 0)
    {
        perror(connectionParameters.serialPort);
        exit(-1);
    }

    struct termios oldtio;
    struct termios newtio;

    // Save current port settings
    if (tcgetattr(fd, &oldtio) == -1)
    {
        perror("tcgetattr");
        exit(-1);
    }

    // Clear struct for new port settings
    memset(&newtio, 0, sizeof(newtio));

    newtio.c_cflag = connectionParameters.baudRate | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;

    // Set input mode (non-canonical, no echo,...)
    newtio.c_lflag = 0;
    newtio.c_cc[VTIME] = 0; // Inter-character timer unused
    newtio.c_cc[VMIN] = 1;  // Blocking read until 5 chars received

    // VTIME e VMIN should be changed in order to protect with a
    // timeout the reception of the following character(s)

    // Now clean the line and activate the settings for the port
    // tcflush() discards data written to the object referred to
    // by fd but not transmitted, or data received but not read,
    // depending on the value of queue_selector:
    //   TCIFLUSH - flushes data received but not read.
    tcflush(fd, TCIOFLUSH);

    // Set new port settings
    if (tcsetattr(fd, TCSANOW, &newtio) == -1)
    {
        perror("tcsetattr");
        exit(-1);
    }

    printf("New termios structure set\n");

    if (connectionParameters.role == LlTx)
    {
        int num_retransmissions = connectionParameters.nRetransmissions;
        int timeout = connectionParameters.timeout;
        unsigned char transmissionBuf[5] = {0}, receiverBuf[5] = {0};

        // Bulding the SET frame
        transmissionBuf[0] = FLAG;
        transmissionBuf[1] = SENDER_ADDRESS;
        transmissionBuf[2] = CONTROL_SET;
        transmissionBuf[3] = SENDER_ADDRESS ^ CONTROL_SET;
        transmissionBuf[4] = FLAG;

        // Retransmission loop
        while (alarmCount < num_retransmissions)
        {
            if (!alarmEnabled)
            {
                int bytes = write(fd, transmissionBuf, sizeof(transmissionBuf));
                if (bytes < 0)
                {
                    printf("Error Sending SET Frame\n");
                    return -1;
                }
                setAlarm(connectionParameters.timeout);
            }
            else
            {
                if (checkUAFrame(fd, receiverBuf, sizeof(receiverBuf)) != 0)
                { // Does it need to be a pointer??? receiver buf == 0) {
                    alarmEnabled = FALSE;
                    continue;
                }
                else
                {
                    printf("UA Frame Received\n");
                    alarmEnabled = FALSE;
                    alarmCount = 0;
                    return 0; /////////////////////////////////////////////////
                    // break;
                }
            }
        }

        if (alarmCount >= num_retransmissions)
        {
            printf("Max number of retransmissions reached\n");
            return -1;
        }
    }
    else
    {
        unsigned char receiverBuf[BUF_SIZE] = {0};
        int bytesRead = read(fd, receiverBuf, sizeof(receiverBuf));

        if (bytesRead < 5)
        { // 5 bytes is the minimum size of a frame
            printf("Error Receiving SET Frame: Invalid Number of Bytes read\n");
            return -1;
        }

        if (state_machine(receiverBuf, bytesRead) != TRUE)
        {
            printf("Error Receiving SET Frame: State Machine Stopped\n");
            return -1;
        }
        else
        {
            printf("SET Frame Received\n");
            unsigned char uaFrame[5] = {0};
            uaFrame[0] = FLAG;
            uaFrame[1] = SENDER_ADDRESS;
            uaFrame[2] = CONTROL_UA;
            uaFrame[3] = SENDER_ADDRESS ^ CONTROL_UA;
            uaFrame[4] = FLAG;

            int response = write(fd, uaFrame, sizeof(uaFrame));
            if (response < 0)
            {
                perror("Error sending UA frame");
                return -1;
            }
        }
    }

    return 100;
}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize)
{
    int frameSize = 0;
    static unsigned char current_control_field = INFO_FRAME_0;
    unsigned char frame[BUF_SIZE] = {0};
    int max_retranfsmissions = 3;
    int attemps = 0;
    bool ack_received = false;

    while (attemps < max_retranfsmissions && !ack_received)
    {
        frameSize = 0;
        frame[frameSize++] = FLAG;
        frame[frameSize++] = SENDER_ADDRESS;
        frame[frameSize++] = current_control_field;
        frame[frameSize++] = SENDER_ADDRESS ^ current_control_field;

        // add data payload and calculte BCC2
        unsigned char BCC2 = 0;

        for (int i = 0; i < bufSize; i++)
        {
            frame[frameSize++] = buf[i];
            BCC2 ^= buf[i];
        }
        frame[frameSize++] = BCC2;
        frame[frameSize++] = FLAG;

        // send the frame over the serial port
        int bytesWritten = write(fd, frame, frameSize);
        if (bytesWritten < 0)
        {
            perror("Error writing frame");
            return -1;
        }

        printf("Frame sent, awaiting acknowledgment...\n");

        // wait for acknowledgment or negative acknowledgment
        unsigned char response[5] = {0};
        int responseSize = read(fd, response, sizeof(response));

        if (responseSize < 5)
        {
            printf("Error: Incomplete responde received\n");
            attemps++;
            continue;
        }

        if (response[0] != FLAG && response[4] != FLAG)
        {
            if (response[2] == CONTROL_RR_0 || response[2] == CONTROL_RR_1)
            {
                printf("Ack (RR) received\n");
                ack_received = true;
                current_control_field = (current_control_field == INFO_FRAME_0) ? INFO_FRAME_1 : INFO_FRAME_0;
            }
            else if (response[2] == CONTROL_REJ_0 || response[2] == CONTROL_REJ_1)
            {
                printf("Negative Ack (REJ) received\n");
                attemps++;
            }
         } else
            {
                printf("Error: Invalid control field in response\n");
                attemps++;
            }
        }
        if (!ack_received)
        {
            printf("Max number of retransmissions reached, frame not sent successfully\n");
            return -1;
        }
        return frameSize;
    
}


// LLREAD notes
/*
    -
    - We have to perform destuffing
    - The receiver will read one packet of an Iframe at a time using a state machine
    - The receiver will only accept frames with the correct address
    - The control field is now used to number the frames

*/

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet)
{
    unsigned char stuffed_iframe[MAX_PAYLOAD_SIZE * 2] = {0};
    unsigned char receiverBuf[1] = {0};
    unsigned stuffed_iframe_size = 0;

    enum states current_state = STATE_START;

    while (TRUE)
    {
        printf("Reading\n");
        while (TRUE)
        {
            int bytesRead = read(fd, receiverBuf, 1);
            if (bytesRead <= 0)
            {
                printf("Read no Bytes From Serial Port\n");
                continue;
            }

            switch (current_state)
            {
            case STATE_START:
                if (receiverBuf[0] == FLAG)
                {
                    current_state = STATE_FLAG_RCV;
                    stuffed_iframe_size++;
                    stuffed_iframe[stuffed_iframe_size] = receiverBuf[0];
                }
                break;

            case STATE_FLAG_RCV:
                if (receiverBuf[0] == FLAG)
                {
                    current_state = STATE_FLAG_RCV; ///////////////Should i send to the first state?
                }
                else
                {
                    current_state = STATE_STOP;
                    stuffed_iframe_size++;
                    stuffed_iframe[stuffed_iframe_size] = receiverBuf[0];
                }
                break;

            case STATE_STOP:
                if (receiverBuf[0] == FLAG)
                {
                    current_state = STATE_START;
                    stuffed_iframe_size++;
                    stuffed_iframe[stuffed_iframe_size] = receiverBuf[0];
                }
                break;
            }
        }
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
