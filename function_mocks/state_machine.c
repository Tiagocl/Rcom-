#include <fcntl.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>

#define BUF_SIZE 256
#define FALSE 0
#define TRUE 1

#define FLAG 0x7E
#define SENDER_ADDRESS 0x03
#define RECEIVER_ADDRESS 0x01
#define CONTROL_SET 0x03
#define CONTROL_UA 0x07
#define BCC1 (SENDER_ADDRESS ^ CONTROL_SET)
#define BCC2 (RECEIVER_ADDRESS ^ CONTROL_UA)

typedef enum states (*State_transition)(void);

long input_counter = 0;
int input_index;


enum states{
    STATE_START,
    STATE_FLAG_RCV,
    STATE_A_RCV,
    STATE_C_RCV,
    STATE_BCC_OK,
    STATE_STOP
} ;

enum events{
    FLAG_EVENT = FLAG,
    SENDER_ADDRESS_EVENT = SENDER_ADDRESS,
    RECEIVER_ADDRESS_EVENT = RECEIVER_ADDRESS,
    CONTROL_SET_EVENT = CONTROL_SET,
    CONTROL_UA_EVENT = CONTROL_UA,
    BCC1_EVENT = BCC1,
    BCC_EVENT2 = BCC2
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


//Need to add something that allows to the detect an invalid state
transition transitions[] = {
    {STATE_START, FLAG_EVENT, goto_FLAG_RCV},
    {STATE_FLAG_RCV, FLAG_EVENT, goto_FLAG_RCV},
    {STATE_FLAG_RCV, SENDER_ADDRESS_EVENT, goto_A_RCV},
    {STATE_A_RCV, FLAG_EVENT, goto_FLAG_RCV},
    {STATE_A_RCV, CONTROL_SET_EVENT, goto_C_RCV},
    {STATE_C_RCV, BCC1_EVENT, goto_BCC_OK},
    {STATE_BCC_OK, FLAG_EVENT, goto_STOP},
    //{STATE_STOP, BCC2, goto_START} //This is the last state, so it should go back to the start?
    //Above is exactly what we wewre talking about, that at this state it can receive anything, but there is no transition for it

};
#define TRANS_COUNT sizeof(transitions)/sizeof(transitions[0])

//Getting next input event
unsigned char get_next_input_event(unsigned char buf[BUF_SIZE + 1]){
    unsigned char input_event = buf[input_index];
    input_counter++;
    input_index = input_counter % 5;
    return input_event;
}

// The state machine
int state_machine(unsigned char buf[BUF_SIZE + 1]){
    
    bool OTHER_RCV = TRUE;  // If it's none of the defined transitions (OTHER_RCV), goto_START
    enum states current_state = STATE_START;/////////////////////////////////////////////////////

    while (current_state != STATE_STOP){
        unsigned char input_event = get_next_input_event(buf);
        for (int i = 0; i < TRANS_COUNT; i++){
            if (transitions[i].origin == current_state){
                if (transitions[i].input == input_event){
                    current_state = transitions[i].transition_function();
                    OTHER_RCV = false;
                    break;
                }
            }
        }
        if (OTHER_RCV){
            current_state = goto_START();
            return -1;
        }
    }

    return 0;
}