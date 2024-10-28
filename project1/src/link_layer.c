<<<<<<< Updated upstream
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
=======
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

//#include <signal.h> //enabling functionality of alarm

#include "link_layer.h"
#include "serial_port.h"
#include "alarm.h"

//Global Variables
int fd;  // Global File descriptor
int txFrameNumber = 0; // Frame number for the transmitter
int rxFrameNumber = 1; // Frame number for the receiver
int lastFrameNumber = -1; // Last frame number fully received
int retransmissionAttempts = 3; // Counter for the number of retransmissions
int retransmissionTimeout = 3; // Timeout for retransmissions

//Bit Mask
#define BIT(n) (1 << n)

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source
#define BUF_SIZE 256

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

typedef enum states (*State_transition)(void);

enum states{
    STATE_START,
    STATE_FLAG_RCV,
    STATE_A_RCV,
    STATE_C_RCV,
    STATE_BCC_OK,
    STATE_STOP
};

enum events{
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
    BCC1_EVENT,                         //need to assign after calculating
    BCC2_EVENT                          //need to assign after calculating
};

typedef struct{
    enum states origin;
    enum events input;
    State_transition transition_function;
} transition;


static enum states goto_START(void){return STATE_START;};
static enum states goto_FLAG_RCV(void){return STATE_FLAG_RCV;};
static enum states goto_A_RCV(void){return STATE_A_RCV;};
static enum states goto_C_RCV(void){return STATE_C_RCV;};
static enum states goto_BCC_OK(void){return STATE_BCC_OK;};
static enum states goto_STOP(void){return STATE_STOP;};


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
#define TRANS_COUNT sizeof(transitions)/sizeof(transitions[0])


//Notes:
/*
Header is composed of Address, Control Field, BCC
Rx State Machine receives a Frame

*/
//requirements of the state machine:

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
int state_machine(unsigned char *buf, int readBytes) {
    
    enum states current_state = STATE_START; /////////////////////////////////////////////////////

    for (int i = 0; i < readBytes && current_state != STATE_STOP; i++) {
        bool OTHER_RCV = TRUE;  // If it's none of the defined transitions (OTHER_RCV), goto_START

        for (int j = 0; j < TRANS_COUNT; j++) {
            if (transitions[j].origin == current_state) {
                if(current_state == STATE_C_RCV){
                    if (i>=2 && buf[i] == (buf[i-1] ^ buf[i-2])){ // BCC1 = A ^ C
                        //BCC1_EVENT = buf[i];///ERROR TRYNG
                        current_state = goto_BCC_OK();
                        OTHER_RCV = FALSE;
                    } 
                }
                if (transitions[j].input == buf[i]) {
                    current_state = transitions[j].transition_function();
                    OTHER_RCV = FALSE;
                    break;
                }
            }
        }

        if (OTHER_RCV){
            if (buf[i] == FLAG) {
                current_state = goto_FLAG_RCV();
                continue;
            } else {   
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

int checkUAFrame(int fd, unsigned char *receiverBuf, size_t bufSize) {
    int response = read(fd, receiverBuf, bufSize);
    if (response < 0) {
        perror("Error receiving UA frame");
        return -1;
    }

    if (response < 5) {
        printf("Error: Incomplete UA frame received\n");
        return -1;
    }

    if (receiverBuf[0] != FLAG || receiverBuf[4] != FLAG) {
        printf("Error: UA frame FLAG field mismatch\n");
        return -1;
    }

    if (receiverBuf[2] != CONTROL_UA) {
        printf("Error: UA frame CONTROL field mismatch\n");
        return -1;
    }

    unsigned char bcc1 = receiverBuf[1] ^ receiverBuf[2];
    if (receiverBuf[3] != bcc1) {
        printf("Error: UA frame BCC1 field mismatch\n");
        return -1;
    }

    return 0;
}


////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters) {
    // Open serial port device for reading and writing and not as controlling tty
    // because we don't want to get killed if linenoise sends CTRL-C.
    
    
    //NONBLOCKING IS MISSIng
    fd = open(connectionParameters.serialPort, O_RDWR | O_NOCTTY);
    if (fd < 0) {
        perror(connectionParameters.serialPort);
        exit(-1);
    }

    struct termios oldtio;
    struct termios newtio;

    // Save current port settings
    if (tcgetattr(fd, &oldtio) == -1) {
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
    if (tcsetattr(fd, TCSANOW, &newtio) == -1) {
        perror("tcsetattr");
        exit(-1);
    }

    printf("New termios structure set\n");

    if(connectionParameters.role == LlTx){
        int num_retransmissions = connectionParameters.nRetransmissions;
        int timeout = connectionParameters.timeout;
        unsigned char transmissionBuf[5] = {0}, receiverBuf[5] = {0};
        retransmissionAttempts = num_retransmissions;
        retransmissionTimeout = timeout;


        //Bulding the SET frame
        transmissionBuf[0] = FLAG;
        transmissionBuf[1] = SENDER_ADDRESS;
        transmissionBuf[2] = CONTROL_SET;
        transmissionBuf[3] = SENDER_ADDRESS ^ CONTROL_SET;
        transmissionBuf[4] = FLAG;

        //Retransmission loop
        while (alarmCount < num_retransmissions) {
            if(!alarmEnabled){
                int bytes = write(fd, transmissionBuf, sizeof(transmissionBuf));
                if(bytes < 5){
                    printf("Error Sending SET Frame\n");
                    return -1;
                }
                setAlarm(connectionParameters.timeout);
            } else {
                if (checkUAFrame(fd, receiverBuf, sizeof(receiverBuf)) != 0) { //Does it need to be a pointer??? receiver buf == 0) {
                    alarmEnabled = FALSE;
                    continue;
                } else {
                    printf("UA Frame Received\n");
                    alarmEnabled = FALSE;
                    alarmCount = 0;
                    return 0; /////////////////////////////////////////////////
                    //break;
                }
            }
        }

        if (alarmCount >= num_retransmissions) {
            printf("Max number of retransmissions reached\n");
            return -1;
        }
        
    } else {
        unsigned char receiverBuf[BUF_SIZE] = {0}; 
        int bytesRead = read(fd, receiverBuf, sizeof(receiverBuf));
        
        if (bytesRead < 5){ // 5 bytes is the minimum size of a frame
            printf("Error Receiving SET Frame: Invalid Number of Bytes read\n");
            return -1;
        }

        if(state_machine(receiverBuf, bytesRead) != TRUE){
            printf("Error Receiving SET Frame: State Machine Stopped\n");
            return -1;
        } else {
            printf("SET Frame Received\n");
            unsigned char uaFrame[5] = {0};
            uaFrame[0] = FLAG;
            uaFrame[1] = SENDER_ADDRESS;
            uaFrame[2] = CONTROL_UA;
            uaFrame[3] = SENDER_ADDRESS ^ CONTROL_UA;
            uaFrame[4] = FLAG;

            int response = write(fd, uaFrame, sizeof(uaFrame));
            if (response < 0) {
                perror("Error sending UA frame");
                return -1;
            }
            
            printf("UA Frame Sent\n");
            return 0;
        }               
    }

    return -1;   //connectionParameters.role;???
}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize)
{
    

    return 0;
}


//LLREAD notes
/*  
    - 
    - We have to perform destuffing 
    - The receiver will read one packet of an Iframe at a time using a state machine
    - The receiver will only accept frames with the correct address 
    - The control field is now used to number the frames

*/
////////////////////////////////////////////////
// LLREAD - AUXILIARY FUNCTIONS
////////////////////////////////////////////////

unsigned char rejectFrameCalculator(unsigned char control_received){
    if (control_received == BIT(0)){
        return CONTROL_REJ_0;
    } else if (control_received == BIT(1)){
        return CONTROL_REJ_1;
    } else {
        printf("Error: REJECT Frame\n");
        return -1;
    }
    return -1;
}

void destuff_frame(const unsigned char *input_frame, int input_len, unsigned char *output_frame, int *output_len) {
    size_t j = 0;
    for (size_t i = 0; i < input_len; i++) {
        if (input_frame[i] == 0x7d) {
            if (i + 1 < input_len) {
                if (input_frame[i + 1] == 0x5e) {
                    output_frame[j++] = 0x7e;
                } else if (input_frame[i + 1] == 0x5d) {
                    output_frame[j++] = 0x7d;
                }
                i++; // Skip the next byte as it is part of the escape sequence
            }
        } else {
            output_frame[j++] = input_frame[i];
        }
    }
    *output_len = j;
}

bool BCC2_validation(const unsigned char *frame, int packet_size) {
    unsigned char bcc2 = frame[packet_size - 2];
    unsigned char bcc2_calculated = 0;
    
    for (int i = 4; i < packet_size - 2; i++) {
        bcc2_calculated ^= frame[i];
    }
    
    if (bcc2 != bcc2_calculated){
        return FALSE;
    } 
    
    return TRUE;
}

int packet_size_calculator(const unsigned char *frame, int frame_size) {
    int l1;
    int l2;
    int packet_size = 0;

    if(frame[4] == 0x01){
        l2 = frame[5];
        l1 = frame[6];  
        packet_size = 256 * l2 + l1;
    } else{
        l1 = frame[6];
        packet_size += l1 + 7;
        
        l2 = frame[packet_size + 1];           // C, T1, L1, FLAG, A, C, BCC
        packet_size += l2 + 4;    // T2, L2, BCC2, FLAG
    }

    return packet_size;
}
////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet)
{
    unsigned char stuffed_iframe[MAX_PAYLOAD_SIZE * 2] = {0};
    //unsigned char *stuffed_iframe = (unsigned char*)malloc(MAX_PAYLOAD_SIZE * 2); //////////// OPTIMIZATIONS FOR LATER
    unsigned char receiverBuf[1] = {0};
    unsigned stuffed_iframe_size = 0;
    int reading_flag = TRUE;
    enum states current_state = STATE_START;
    
    while (reading_flag){
        printf("Reading\n");
        int bytesRead = read(fd, receiverBuf, 1);
        if (bytesRead <= 0){
            printf("Read no Bytes From Serial Port\n");
            continue;
        }

        switch (current_state){
                case STATE_START:
                    if (receiverBuf[0] == FLAG){
                        current_state = STATE_FLAG_RCV;
                        stuffed_iframe_size++;
                        stuffed_iframe[stuffed_iframe_size] = receiverBuf[0];
                    }
                    break;

                case STATE_FLAG_RCV:
                    if (receiverBuf[0] == FLAG){
                        current_state = STATE_FLAG_RCV; ///////////////Should i send to the first state?
                    } else {
                        current_state = STATE_STOP;
                        stuffed_iframe_size++;
                        stuffed_iframe[stuffed_iframe_size] = receiverBuf[0];
                    }
                    break;

                case STATE_STOP:
                        stuffed_iframe_size++;
                        stuffed_iframe[stuffed_iframe_size] = receiverBuf[0];

                    if (receiverBuf[0] == FLAG){
                        current_state = STATE_START;
                        reading_flag = FALSE;
                    }
                    break;
                default:
                    printf("Error: LLOPEN Switch\n");
                    return -1;
            }
        } 

            //verifying first fields in stuffed frame by checking the address and control fields and BCC1
            //control has the frame number so if there is an error with a given frame i need to send the according REJ(n) frame
            //to calculate the REJ frame i need to know the frame number of the frame that was received
            //if the frame number is 0, the REJ frame will have the control field with the frame number 1 and vice versa
            unsigned char bcc1 = stuffed_iframe[1] ^ stuffed_iframe[2];

            if(bcc1 != stuffed_iframe[3] || stuffed_iframe[2] == rxFrameNumber){        //////////////// I need to discern that a dulicated fame is ignores and sent the RR command back
                printf("Error: BCC1 Mismatch\n");
                printf("Error: OR\n");
                printf("Error: Frame Number Mismatch\n");

                unsigned char rej_command[5] = {0};
                rej_command[0] = FLAG;
                rej_command[1] = RECEIVER_ADDRESS;
                rej_command[2] = rejectFrameCalculator(stuffed_iframe[2]);
                rej_command[3] = rej_command[1] ^ rej_command[2];
                rej_command[4] = FLAG;

                printf("Sending REJ Frame\n");
                write(fd, rej_command, sizeof(rej_command));
                
                return -1;
            }

            //destuffing the stuffed frame
            unsigned char destuffed_iframe[MAX_PAYLOAD_SIZE] = {0};  //////////////// OPTIMIZATIONS FOR LATER with malloc and realoc
            int destuffed_iframe_size = 0;
            destuff_frame(stuffed_iframe, stuffed_iframe_size, destuffed_iframe, &destuffed_iframe_size);

            //verifying the BCC2
            int packet_size = packet_size_calculator(destuffed_iframe, destuffed_iframe_size);

            if(BCC2_validation(destuffed_iframe, packet_size)){
                if(destuffed_iframe[4] == 0x01){
                    
                    if(destuffed_iframe[2] == rxFrameNumber){
                        printf("Duplicated Frame received. Ignoring and sending RR(%d)\n", rxFrameNumber);
                    } else {
                        lastFrameNumber = destuffed_iframe[2];
                    }

                }
                    
                unsigned char rr_command[5] = {0};
                rr_command[0] = FLAG;
                rr_command[1] = SENDER_ADDRESS;
                rr_command[2] = rxFrameNumber == 0 ? CONTROL_RR_1 : CONTROL_RR_0;
                rr_command[3] = rr_command[1] ^ rr_command[2];                    rr_command[4] = FLAG;
                int valid = write(fd, rr_command, sizeof(rr_command));
                if (valid < 0){
                    printf("Error: Writing RR Frame\n");
                }
                return -1;
                
           
            } else{
                unsigned char rej_command[5] = {0};
                rej_command[0] = FLAG;
                rej_command[1] = SENDER_ADDRESS;
                rej_command[2] = rejectFrameCalculator(txFrameNumber);
                rej_command[3] = rej_command[1] ^ rej_command[2];
                rej_command[4] = FLAG;
                int valid = write(fd, rej_command, sizeof(rej_command));
                if (valid < 0){
                    printf("Error: Writing REJ Frame\n");
                }
                
                return -1;
            }

            memset(packet,0,sizeof(*packet)); //Added * to clear warning
            for (int i = 4; i < packet_size - 6; i++){  // 6 bytes are the header and trailer
                packet[i - 4] = destuffed_iframe[i];
            }

            txFrameNumber = !txFrameNumber;
            rxFrameNumber = !rxFrameNumber;

    return packet_size;
}

            //Some caveats that need to be checked:
            //
            //If the frame is a new frame, the receiver should send a RR(n) frame and deliver the data to the application layer
            //If the frame is a duplicate, the receiver should send a RR(n) frame and discard the data

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

            
            */
            // IMPORTANT DONT FORGET TO UPDATE THE FRAME NUMBER AFTER SENDING THE RR FRAME and etc... those 3 variables are important



////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose(int showStatistics)
{
    unsigned char transmissionBuf[5] = {0}, receiverBuf[5] = {0};
    transmissionBuf[0] = FLAG;
    transmissionBuf[1] = SENDER_ADDRESS;
    transmissionBuf[2] = CONTROL_DISC;
    transmissionBuf[3] = transmissionBuf[1] ^ transmissionBuf[2];
    transmissionBuf[4] = FLAG;

        while (alarmCount < retransmissionAttempts) {
            //unsigned char event_byte[1] = {0};

            if(!alarmEnabled){
                printf("Sending Tx DISC Frame\n");
                int bytes = write(fd, transmissionBuf, sizeof(transmissionBuf));
                if(bytes < 5){
                    printf("Error Sending SET Frame\n");
                    return -1;
                }
                setAlarm(retransmissionTimeout);
            }

            int response = read(fd, receiverBuf, 5);
            
            if (response == 5) {
                printf("Receiving Rx DISC Frame\n");
                if(state_machine(receiverBuf, response) == TRUE){
                    if(receiverBuf[2] == CONTROL_DISC){
                        printf("Rx DISC Frame Received\n");
                        transmissionBuf[0] = FLAG;
                        transmissionBuf[1] = SENDER_ADDRESS;
                        transmissionBuf[2] = CONTROL_UA;
                        transmissionBuf[3] = transmissionBuf[1] ^ transmissionBuf[2];
                        transmissionBuf[4] = FLAG;
                        if(write(fd, transmissionBuf, sizeof(transmissionBuf)) < 5){
                            printf("Error Sending UA Frame\n");
                            return -1;
                        }
                        printf("Sending UA Frame\n");
                        break;
                    } else if (receiverBuf[2] == CONTROL_UA){
                        printf("Rx UA Frame Received\n");
                        break;
                    }     
                }
            }
        }

        if (alarmCount >= retransmissionAttempts) {
            printf("Max number of retransmissions reached\n");
            return -1;
        }

        alarmEnabled = FALSE;
        alarmCount = 0;

    
    int clstat = closeSerialPort();
    return clstat;
}
>>>>>>> Stashed changes
