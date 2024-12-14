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

//Extern
extern int pinguim;

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

//Stuffing
#define ESCAPE 0x7D
#define XOR_BYTE 0x20

typedef enum states (*State_transition)(void);

LinkLayerRole role;

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




int checkUAFrame(int fd, unsigned char *receiverBuf, size_t buff_size) {
    
    //printf("Bytes checkUAFrame Read: %lu\n", buff_size);

    //int response = read(fd, receiverBuf, bufSize);

    if (buff_size < 0) {
        perror("Error receiving UA frame");
        return FALSE;
    }

    if (buff_size < 5) {
        printf("Error: Incomplete UA frame received\n");
        return FALSE;
    }

    for (int i = 0; i < 5; i++){
        printf("Check UA ReceiverBuf[%d]: %02x\n", i, receiverBuf[i]);
    }

    if (receiverBuf[0] != FLAG || receiverBuf[4] != FLAG) {
        printf("Error: UA frame FLAG field mismatch\n");
        return FALSE;
    }

    

    if (receiverBuf[2] != CONTROL_UA) {
        printf("Error: UA frame CONTROL field mismatch\n");
        return FALSE;
    }

    unsigned char bcc1 = receiverBuf[1] ^ receiverBuf[2];
    if (receiverBuf[3] != bcc1) {
        printf("Error: UA frame BCC1 field mismatch\n");
        return FALSE;
    }

    return TRUE;
}


////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters) {//resonsible for establishing connection between over a serial port 


    fd = openSerialPort(connectionParameters.serialPort, connectionParameters.baudRate);//opens serial port 

    if (fd < 0) { //check if opened successfully
        printf("Error: Opening Serial Port\n");
        return -1;
    }
   
    role = connectionParameters.role;

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

        //printf("SET Frame Built\n");
        //printf("Set Frame: %x %x %x %x %x\n", transmissionBuf[0], transmissionBuf[1], transmissionBuf[2], transmissionBuf[3], transmissionBuf[4]);

        //Retransmission loop
        while (alarmCount < num_retransmissions) {
            if(!alarmEnabled){
                int bytes = write(fd, transmissionBuf, sizeof(transmissionBuf));
                //printf("Bytes Sent: %d\n", bytes);
                //printf("Bytes Sent: %02x %02x %02x %02x %02x\n", transmissionBuf[0], transmissionBuf[1], transmissionBuf[2], transmissionBuf[3], transmissionBuf[4]);
                if(bytes < 5){
                    printf("Error Sending SET Frame\n");
                    return -1;
                }
                setAlarm(connectionParameters.timeout);
            } else {
                int receiver_buff_size = 0;
                for (int i = 0; i < 5; i++){
                    if(read(fd, &receiverBuf[i], 1) != 0){  //?? do i need that on receiverBuf?
                        receiver_buff_size++;
                    } 
                }
/*
 */               
                int checkUA = checkUAFrame(fd, receiverBuf, receiver_buff_size);
                //printf("CheckUA: %d\n", checkUA);
                if (checkUA == 0) { //Does it need to be a pointer??? receiver buf == 0) {
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
        unsigned char receiverBuf[5] = {0}; 
        int bytesRead = 0;
        
        for (int i = 0; i < 5; i++){
            read(fd, &receiverBuf[i], 1);
            bytesRead++;
        }


        printf("Bytes Read: %d\n", bytesRead);
        printf("Receiver buf size: %lu\n", sizeof(receiverBuf));
        if (bytesRead < 5){ // 5 bytes is the minimum size of a frame
            printf("Error Receiving SET Frame: Invalid Number of Bytes read\n");
            return -1;
        }
        /*
        int bytesRead = read(fd, receiverBuf, sizeof(receiverBuf));
        printf("Bytes Read: %d\n", bytesRead);
        if (bytesRead < 5){ // 5 bytes is the minimum size of a frame
            printf("Error Receiving SET Frame: Invalid Number of Bytes read\n");
            return -1;
        }
        */

        if(state_machine(receiverBuf, bytesRead) != TRUE){
            printf("Error Receiving SET Frame: State Machine Stopped\n");
            return -1;
        } else {
            //printf("SET Frame Received\n");
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
            
            //printf("UA Frame Sent\n");
            return 0;
        }               
    }

    return -1;   //connectionParameters.role;???
}

// LLWRITE notes
/*
    -
    - We have to perform byte stuffing

*/
int performByteStuffing(const unsigned char *input, int inputSize, unsigned char *output) {
    int outputSize = 0; 
// if a byte in the payload matches one ofth special characters FLAG or ESCAPE
// it is replaced with a sequence of two bytes to prevent confuion with contrl characters
    for(int i=0; i<inputSize;i++) {
        unsigned char byte = input[i];

        if(byte == FLAG) {
            output[outputSize++] = ESCAPE;
            output[outputSize++] = FLAG ^ XOR_BYTE;
        }
        else if (byte == ESCAPE) {
            output[outputSize++] = ESCAPE; 
            output[outputSize++] = ESCAPE ^ XOR_BYTE;
        }
        else {
            output[outputSize++] = byte;
        }
    }
    return outputSize;
}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize){
    int frameSize = 0;
    static unsigned char current_control_field = INFO_FRAME_0;
    unsigned char frame[MAX_PAYLOAD_SIZE *2] = {0};
    int max_retranfsmissions = 3;
    int attempts = 0;
    bool ack_received = false;

    
    while (attempts < max_retranfsmissions && !ack_received){
        frameSize = 0;
        frame[frameSize++] = FLAG;
        frame[frameSize++] = SENDER_ADDRESS;
        frame[frameSize++] = current_control_field;
        frame[frameSize++] = SENDER_ADDRESS ^ current_control_field;

        // add data payload and calculte BCC2
        unsigned char stuffedData[MAX_PAYLOAD_SIZE * 2] = {0};
        unsigned char BCC2 = 0;

        for (int i = 0; i < bufSize; i++){
            BCC2 ^= buf[i];
        }


        int stuffedDataSize = performByteStuffing(buf,bufSize,stuffedData);
      
        printf("\n");
        

        //copy stuffed data to the frame 
        memcpy(&frame[frameSize],stuffedData,stuffedDataSize);
        frameSize += stuffedDataSize;

       


        //add BCC2 to frame and apply stuffing to BCC2 
        unsigned char stuffedBCC2[2];
        int stuffedBCC2Size = performByteStuffing(&BCC2,1,stuffedBCC2);

        memcpy(&frame[frameSize],stuffedBCC2,stuffedBCC2Size);
        frameSize += stuffedBCC2Size;

        

        frame[frameSize++] = FLAG;

        
        // send the frame over the serial port
        int bytesWritten = write(fd, frame, frameSize);
        if (bytesWritten < 0){
            perror("Error writing frame");
            return -1;
        }

    //Till here GOOD

        //printf("Frame sent, awaiting acknowledgment...\n");
        printf("Frame sent, awaiting acknowledgment...\n");
        // wait for acknowledgment or negative acknowledgment

        printf("Entering new State Machine\n");
        unsigned char byte;
        enum states state = STATE_START;
        int responseSize = 0;
        unsigned char response[5] = {0};

        while (responseSize < 5){
            if (read(fd, &byte, 1) > 0){
                printf("Byte Read: %02x\n", byte);
                switch (state) {
                    case STATE_START:
                        if (byte == FLAG){state = STATE_FLAG_RCV; response[responseSize++] = byte;} 
                        break;
                    case STATE_FLAG_RCV:
                        if (byte == SENDER_ADDRESS) {state = STATE_A_RCV; response[responseSize++] = byte;}
                        else if (byte != FLAG) {state = STATE_START; responseSize = 0;}
                        break;
                    case STATE_A_RCV:
                        if (byte == CONTROL_RR_0 || byte == CONTROL_RR_1) {state = STATE_C_RCV; response[responseSize++] = byte;}
                        else if (byte == FLAG) {state = STATE_FLAG_RCV; responseSize = 1;}
                        else {state = STATE_START, responseSize = 0;};
                        break;
                    case STATE_C_RCV:
                        if (byte == (CONTROL_RR_0 ^ SENDER_ADDRESS) || byte == (CONTROL_RR_1 ^ SENDER_ADDRESS)) {
                            state = STATE_BCC_OK; response[responseSize++] = byte;}
                        else if (byte == FLAG) {state = STATE_FLAG_RCV; responseSize = 1;}
                        else {state = STATE_START; responseSize = 0;}
                        break;
                    case STATE_BCC_OK:
                        if (byte == FLAG) {state = STATE_STOP; response[responseSize++] = byte;}
                        else {state = STATE_START; responseSize = 0;}
                        break;
                    default: 
                        break;
                }
            }
        }
        

/*       
        printf("Response Size: %d\n", responseSize);
        printf("Response: %02x %02x %02x %02x %02x\n", response[0], response[1], response[2], response[3], response[4]);
        //int responseSize = read(fd, response, sizeof(response));
*/

        if (responseSize < 5){
            printf("Error: Incomplete responde received\n");
            attempts++;
            continue;
        }

        if (response[0] == FLAG && response[4] == FLAG){
            if (response[2] == CONTROL_RR_0 || response[2] == CONTROL_RR_1){
                printf("Ack (RR) received\n");
                ack_received = true;
                current_control_field = (current_control_field == INFO_FRAME_0) ? INFO_FRAME_1 : INFO_FRAME_0;
                resetAlarm();
            }
            else if (response[2] == CONTROL_REJ_0 || response[2] == CONTROL_REJ_1){
                printf("Negative Ack (REJ) received\n");
                attempts++;
            }

        } else {
            printf("Error: Invalid control field in response\n");
            attempts++;
        }

        memset(response, 0, sizeof(response));
        responseSize = 0;
    }

    if (!ack_received){
        printf("Max number of retransmissions reached, frame not sent successfully\n");
        return -1;
    }
    
    return frameSize;
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
    if (control_received == 0x00){
        return CONTROL_REJ_0;
    } else if (control_received == 0x80){
        return CONTROL_REJ_1;
    } else {
        printf("Error: REJECT Frame\n");
    }
    return -1;
}


unsigned char readyFrameCalculator(unsigned char control_received){
    if (control_received == 0x00){
        return CONTROL_RR_1;
    } else if (control_received == 0x80){
        return CONTROL_RR_0;
    } else {
        printf("Error: Receiver Ready Frame\n");
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
    unsigned char bcc2_calculated = frame[4];
    
    //printf("BCC2 Packet Size: %d\n", packet_size);

    for (int i = 5; i < packet_size - 2; i++) {
        bcc2_calculated ^= frame[i];
    }
    
    //printf("?BCC2: 0x%02X\n", bcc2_calculated);

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

int checkDISCFrame( unsigned char *receiver_buff, int buff_size){

    if (buff_size < 0) {
        perror("Error receiving DISC frame");
        return FALSE;
    }

    if (buff_size < 5) {
        printf("Error: Incomplete DISC frame received\n");
        return FALSE;
    }
    printf("Check DISC ReceiverBuf: ");
    for (int i = 0; i < 5; i++){
        printf("%02x ", receiver_buff[i]);
    }
    printf("\n");

    if (receiver_buff[0] != FLAG || receiver_buff[4] != FLAG) {
        printf("Error: DISC frame FLAG field mismatch\n");
        return FALSE;
    }

    if (receiver_buff[2] != CONTROL_DISC) {
        printf("Error: DISC frame CONTROL field mismatch\n");
        return FALSE;
    }

    unsigned char bcc1 = receiver_buff[1] ^ receiver_buff[2];
    if (receiver_buff[3] != bcc1) {
        printf("Error: DISC frame BCC1 field mismatch\n");
        return FALSE;
    }

    return TRUE;
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet) {
    unsigned char stuffed_iframe[MAX_PAYLOAD_SIZE * 2] = {0};
    //unsigned char *stuffed_iframe = (unsigned char*)malloc(MAX_PAYLOAD_SIZE * 2); //////////// OPTIMIZATIONS FOR LATER
    unsigned char receiverBuf[1] = {0};
    unsigned stuffed_iframe_size = 0;
    int reading_flag = TRUE;
    enum states current_state = STATE_START;
    long int total_bytes_read = 0;
    
    //printf("Started Reading\n");

    while (reading_flag){
        
        int bytesRead = read(fd, receiverBuf, 1);
        if (bytesRead <= 0){
            printf("Read no Bytes From Serial Port\n");
            continue;
        }
        total_bytes_read += bytesRead;

        switch (current_state){
            case STATE_START:
                printf("STATE_START\n");
                if (receiverBuf[0] == FLAG){
                    current_state = STATE_FLAG_RCV;
                    stuffed_iframe[stuffed_iframe_size++] = receiverBuf[0];
                }
                break;

            case STATE_FLAG_RCV:
                //printf("STATE_FLAG_RCV\n");
                if (receiverBuf[0] == FLAG){
                    printf("FLAG RCV\n");
                    current_state = STATE_FLAG_RCV; ///////////////Should i send to the first state?
                } else {
                    //printf("FLAG NOT RCV STARTING TO REGISTER DATA BYTES\n");
                    current_state = STATE_STOP;
                    stuffed_iframe[stuffed_iframe_size++] = receiverBuf[0];
                }
                break;

            case STATE_STOP:
                    stuffed_iframe[stuffed_iframe_size++] = receiverBuf[0];
                    
                if (receiverBuf[0] == FLAG){
                    //printf("STATE_STOP\n");
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
/*    
    printf("stuff_iframe[0]: 0x%02X\n", stuffed_iframe[0]);
    printf("stuff_iframe[1]: 0x%02X\n", stuffed_iframe[1]);
    printf("stuff_iframe[2]: 0x%02X\n", stuffed_iframe[2]);
    printf("BCC1 > 0x%02X\n", bcc1);
*/


    if(bcc1 != stuffed_iframe[3] || stuffed_iframe[2] == rxFrameNumber){        //////////////// I need to discern that a dulicated fame is ignores and sent the RR command back
        printf("Error: BCC1 Mismatch\n");
        printf("Error: OR\n");
        printf("Error: Frame Number Mismatch\n");
        printf("Error: Frame Number > %d\n", stuffed_iframe[2]);

        unsigned char rej_command[5] = {0};
        rej_command[0] = FLAG;
        rej_command[1] = RECEIVER_ADDRESS;
        rej_command[2] = rejectFrameCalculator(stuffed_iframe[2]);
        rej_command[3] = rej_command[1] ^ rej_command[2];
        rej_command[4] = FLAG;

        printf("Sending REJ Frame\n");
        printf("rej_command: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n", rej_command[0], rej_command[1], rej_command[2], rej_command[3], rej_command[4]);
        printf("sizeof(rej_command): %lu\n", sizeof(rej_command));
/*        
        if (tcflush(fd, TCIOFLUSH) == -1) {
            perror("tcflush");
            close(fd);
            return EXIT_FAILURE;
        }
*/

        write(fd, rej_command, sizeof(rej_command));
        
        return -1;
    }

    //destuffing the stuffed frame
    unsigned char destuffed_iframe[MAX_PAYLOAD_SIZE] = {0};  //////////////// OPTIMIZATIONS FOR LATER with malloc and realoc
    int destuffed_iframe_size = 0;
    destuff_frame(stuffed_iframe, stuffed_iframe_size, destuffed_iframe, &destuffed_iframe_size);

    //verifying the BCC2
    printf("_______________________________\n");
    printf("Pre Destuffing Frame Size: %d\n", stuffed_iframe_size);
    printf("Actual pre destuffing frame size: %lu\n", sizeof(stuffed_iframe));
    printf("Post Destuffing Frame Size: %d\n", destuffed_iframe_size);
    printf("Actual post destuffing frame size: %lu\n", sizeof(destuffed_iframe));
    int packet_size = packet_size_calculator(destuffed_iframe, destuffed_iframe_size);
    printf("Packet Size: %d\n", packet_size);
    printf("_______________________________\n");

/*
    printf("Destuffed iframe:\n");
    for (int i = 0; i < destuffed_iframe_size; i++) {
        printf("0x%02X ", destuffed_iframe[i]);
    }
    printf("\n");

    printf("check disc result: %d\n", checkDISCFrame(destuffed_iframe, destuffed_iframe_size));
*/

    if(BCC2_validation(destuffed_iframe, destuffed_iframe_size)){     //////////////// Changed 2nd argument from packet size to destuffed_iframe_size
        if(destuffed_iframe[4] == 0x01){
            printf("DO NOT ENTER REPLICATION PART IS WRONG ANDbusted\n");
            if(destuffed_iframe[2] == rxFrameNumber){
                printf("Duplicated Frame received. Ignoring and sending RR(%d)\n", rxFrameNumber);
            } else {
                lastFrameNumber = destuffed_iframe[2];
            }

        }
   /*     
        printf("BCC2 Validation Passed\n");
        // Print the frame number in hexadecimal format
        //printf("Frame Number (Decimal): %d\n", destuffed_iframe[2]);
        printf("Frame Number (Hex): 0x%02X\n", destuffed_iframe[2]);
*/
        unsigned char rr_command[5] = {0};
        rr_command[0] = FLAG;
        rr_command[1] = SENDER_ADDRESS;
        rr_command[2] = readyFrameCalculator(destuffed_iframe[2]);           //////////////// Changed rxFrameNumber to txFrameNumber ptobably not the best idea
        rr_command[3] = rr_command[1] ^ rr_command[2];                    
        rr_command[4] = FLAG;

/*
        printf("Sending RR Frame\n");
        // Print the rr_command in hex
        printf("RR Command:\n");
        for (int i = 0; i < 5; i++) {
            printf("0x%02X ", rr_command[i]);
        }
        printf("\n");
*/
        int valid = write(fd, rr_command, sizeof(rr_command));
        if (valid < 0){
            printf("Error: Writing RR Frame\n");
        }

        //printf("RR Frame Sent missing something here\n");
        //printf("NUMBER OF BYTES READ: %ld\n", total_bytes_read);
        //return -1;      //Isnt this supposed to be 0 or 5
        
    } else if (checkDISCFrame(destuffed_iframe, destuffed_iframe_size)){
        printf("DISC Frame Received\n");
        unsigned char ua_command[5] = {0};
        for (int i = 0; i < 5; i++){
            int read_thing = read(fd, &ua_command[i], 1);
            if (read_thing < 0){
                printf("Error: Reading UA Frame\n");
            }
        }
        if (ua_command[2] == CONTROL_UA){
            printf("UA Frame Received\n");
            return 0; 
        } else {
            printf("Error: UA Frame Not Read\n");
            return -1;
        }
        
    


/*
        unsigned char disc_command[5] = {0};
        disc_command[0] = FLAG;
        disc_command[1] = SENDER_ADDRESS;
        disc_command[2] = CONTROL_DISC;
        disc_command[3] = disc_command[1] ^ disc_command[2];
        disc_command[4] = FLAG;
        int valid = write(fd, disc_command, sizeof(disc_command));
        if (valid < 0){
            printf("Error: Writing UA Frame\n");
        printf("???Segmentation Fault????\n");
              ///????????????????????
*/
        } else {
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
        printf("Error: here\n");
        return -1;
    }

    //memset(packet,0,sizeof(*packet)); //Added * to clear warning
    for (int i = 4; i < destuffed_iframe_size - 2; i++){  // 6 bytes are the header and trailer  //////////////// OPTIMIZATIONS FOR LATER//changed packet_size to destuffed_iframe_size
        packet[i - 4] = destuffed_iframe[i];
    }

    // Print the packet in hex
/*            
    printf("Packet:\n");
    for (int i = 0; i < destuffed_iframe_size - 6; i++) {
        printf("0x%02X ", packet[i]);
    }
    printf("\n");
*/
    txFrameNumber = !txFrameNumber;
    rxFrameNumber = !rxFrameNumber;

    return  destuffed_iframe_size - 6;     //packet_size;
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
int llclose(int showStatistics){
    int clstat;   
        if(role == LlTx){
            unsigned char transmissionBuf[5] = {0}, receiverBuf[5] = {0};
            transmissionBuf[0] = FLAG;
            transmissionBuf[1] = SENDER_ADDRESS;
            transmissionBuf[2] = CONTROL_DISC;
            transmissionBuf[3] = transmissionBuf[1] ^ transmissionBuf[2];
            transmissionBuf[4] = FLAG;

            while (alarmCount < retransmissionAttempts) {
                //unsigned char event_byte[1] = {0};
                printf("Alarm Enabled: %d\n", alarmEnabled);
                printf("Alarm Count: %d\n", alarmCount);
                printf("HELLO\n");
                if(!alarmEnabled){
                    printf("Sending Tx DISC Frame\n");
                    int bytes = write(fd, transmissionBuf, sizeof(transmissionBuf));
                    if(bytes < 5){
                        printf("Error Sending DISC Frame\n");
                        return -1;
                    }
                    setAlarm(retransmissionTimeout);
                }

                
                int response = 0;
                for(int i = 0; i < 5; i++){
                    if(read(fd, &receiverBuf[i], 1) == 1){
                        response++;
                    }
                }
                
                printf("Bytes Read on DISC Response: %d\n", response);
                if (response == 5) {
                    printf("Receiving Rx DISC Frame\n");
                    int state_machine_response = state_machine(receiverBuf, response);
                    printf("State Machine Response: %d\n", state_machine_response);
                    if((state_machine_response) == TRUE){
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
                        } else if (receiverBuf[2] == CONTROL_UA){       // DONT THINK THIS IS NEEDED
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

        
            clstat = closeSerialPort();
            printf("Serial port closed\n");
            printf("clstat: %d\n", clstat);
        } else{
            unsigned char transmissionBuf[5] = {0}, receiverBuf[5] = {0};
            transmissionBuf[0] = FLAG;
            transmissionBuf[1] = SENDER_ADDRESS;
            transmissionBuf[2] = CONTROL_DISC;
            transmissionBuf[3] = transmissionBuf[1] ^ transmissionBuf[2];
            transmissionBuf[4] = FLAG;

            do
            {
                int response = 0;
                for(int i = 0; i < 5; i++){
                    if(read(fd, &receiverBuf[i], 1) == 1){
                        response++;
                    }
                }
                
            } while ((receiverBuf[1] ^ receiverBuf[2]) != receiverBuf[3] || receiverBuf[2] != CONTROL_DISC);

            printf("WHILE?\n");
            
            



            while (alarmCount < retransmissionAttempts) {
                if(!alarmEnabled){
                    printf("Sending Tx DISC Frame\n");
                    int bytes = write(fd, transmissionBuf, sizeof(transmissionBuf));
                    if(bytes < 5){
                        printf("Error Sending DISC Frame\n");
                        return -1;
                    }
                    setAlarm(retransmissionTimeout);
                }

                
                int response = 0;
                for(int i = 0; i < 5; i++){
                    if(read(fd, &receiverBuf[i], 1) == 1){
                        response++;
                    }
                }
                
                printf("Bytes Read on ua Response: %d\n", response);
                if (response == 5) {
                    printf("Receiving tx ua Frame\n");
                    int state_machine_response = state_machine(receiverBuf, response);
                    printf("State Machine Response: %d\n", state_machine_response);
                    if((state_machine_response) == TRUE){
                        if(receiverBuf[2] == CONTROL_UA){
                            printf("tx UA Frame Received\n");
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

        
            clstat = closeSerialPort();
            printf("Serial port closed\n");
            printf("clstat: %d\n", clstat);

        }
        return clstat;
}
