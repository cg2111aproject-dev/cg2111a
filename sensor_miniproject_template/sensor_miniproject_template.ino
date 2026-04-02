/*
 * sensor_miniproject_template.ino
 * Studio 13: Sensor Mini-Project
 *
 * This sketch is split across three files in this folder:
 *
 *   packets.h        - TPacket protocol: enums, struct, framing constants.
 *                      Must stay in sync with pi_sensor.py.
 *
 *   serial_driver.h  - Transport layer.  Set USE_BAREMETAL_SERIAL to 0
 *                      (default) for the Arduino Serial path that works
 *                      immediately, or to 1 to use the bare-metal USART
 *                      driver (Activity 1).  Also contains the
 *                      sendFrame / receiveFrame framing code.
 *
 *   sensor_miniproject_template.ino  (this file)
 *                    - Application logic: packet helpers, E-Stop state
 *                      machine, color sensor, setup(), and loop().
 */




#include "packets.h"
#include "serial_driver.h"
#include <AFMotor.h>
#include <avr/io.h>
#include <avr/interrupt.h>




// =============================================================
// Motor definitions
// =============================================================
#define FRONT_LEFT   2
#define FRONT_RIGHT  1
#define BACK_LEFT    3
#define BACK_RIGHT   4




AF_DCMotor motorFL(FRONT_LEFT);
AF_DCMotor motorFR(FRONT_RIGHT);
AF_DCMotor motorBL(BACK_LEFT);
AF_DCMotor motorBR(BACK_RIGHT);




static uint8_t motorSpeed = 200;




typedef enum { MSTOP, GO, BACK, CCW, CW } DriveDir;




static void move(uint8_t speed, DriveDir direction) {
    motorFL.setSpeed(speed);
    motorFR.setSpeed(speed);
    motorBL.setSpeed(speed);
    motorBR.setSpeed(speed);
    switch (direction) {
        case GO:
            motorFL.run(BACKWARD);  motorFR.run(FORWARD);
            motorBL.run(BACKWARD); motorBR.run(FORWARD);
            break;
        case BACK:
            motorFL.run(FORWARD); motorFR.run(BACKWARD);
            motorBL.run(FORWARD); motorBR.run(BACKWARD);
            break;
        case CW:
            motorFL.run(BACKWARD); motorFR.run(BACKWARD);
            motorBL.run(BACKWARD);  motorBR.run(BACKWARD);
            break;
        case CCW:
            motorFL.run(FORWARD);  motorFR.run(FORWARD);
            motorBL.run(FORWARD); motorBR.run(FORWARD);
            break;
        case MSTOP:
        default:
            motorFL.run(RELEASE); motorFR.run(RELEASE);
            motorBL.run(RELEASE); motorBR.run(RELEASE);
            break;
    }
}




// =============================================================
// Claw servo definitions
// =============================================================
#define BASE_LOWER_LIMIT      0
#define BASE_UPPER_LIMIT      180
#define SHOULDER_LOWER_LIMIT  50
#define SHOULDER_UPPER_LIMIT  135
#define ELBOW_LOWER_LIMIT     105
#define ELBOW_UPPER_LIMIT     180
#define GRIPPER_LOWER_LIMIT   90
#define GRIPPER_UPPER_LIMIT   105




#define BASE_PIN     (1 << 0)
#define SHOULDER_PIN (1 << 1)
#define ELBOW_PIN    (1 << 2)
#define GRIPPER_PIN  (1 << 3)




#define B_CHECKPOINT    0
#define S_CHECKPOINT    10000
#define E_CHECKPOINT    20000
#define G_CHECKPOINT    30000
#define TOP_CHECKPOINT  39999




#define STARTTICKS  3000
#define START_TPP   20




static int          clawStage   = 0;
static unsigned int clawTPP     = START_TPP;
static unsigned int B_currticks = STARTTICKS, S_currticks = STARTTICKS;
static unsigned int E_currticks = STARTTICKS, G_currticks = STARTTICKS;
static unsigned int B_target    = STARTTICKS, S_target    = STARTTICKS;
static unsigned int E_target    = STARTTICKS, G_target    = STARTTICKS;




static inline unsigned int degToTicks(int deg) {
    return (unsigned int)(deg * 22.22222f + 0.5f) + 1000;
}




static inline unsigned int mpdToTPP(unsigned int mpd) {
    unsigned int t = (unsigned int)(444.44444f / mpd + 0.5f);
    return (t == 0) ? 1 : t;
}




// Timer5 Compare B ISR — claw servo pulse staging
ISR(TIMER5_COMPB_vect) {
    switch (clawStage) {
        case 0:
            PORTC |= BASE_PIN;
            OCR5B += B_currticks;
            break;
        case 1:
            PORTC &= ~BASE_PIN;
            OCR5B  = S_CHECKPOINT;
            break;
        case 2:
            PORTC |= SHOULDER_PIN;
            OCR5B += S_currticks;
            break;
        case 3:
            PORTC &= ~SHOULDER_PIN;
            OCR5B  = E_CHECKPOINT;
            break;
        case 4:
            PORTC |= ELBOW_PIN;
            OCR5B += E_currticks;
            break;
        case 5:
            PORTC &= ~ELBOW_PIN;
            OCR5B  = G_CHECKPOINT;
            break;
        case 6:
            PORTC |= GRIPPER_PIN;
            OCR5B += G_currticks;
            break;
        case 7:
            PORTC &= ~GRIPPER_PIN;
            OCR5B     = B_CHECKPOINT;
            clawStage = -1;
            break;
    }
    clawStage++;
}




// Timer5 Compare A ISR — claw servo lerp tick every 20 ms
ISR(TIMER5_COMPA_vect) {
    #define LERP_ONE(cur, tgt, step) \
        do { \
            if ((cur) < (tgt)) { (cur) += (step); if ((cur) > (tgt)) (cur) = (tgt); } \
            else               { (cur) -= (step); if ((cur) < (tgt)) (cur) = (tgt); } \
        } while (0)
    LERP_ONE(B_currticks, B_target, clawTPP);
    LERP_ONE(S_currticks, S_target, clawTPP);
    LERP_ONE(E_currticks, E_target, clawTPP);
    LERP_ONE(G_currticks, G_target, clawTPP);
}




// =============================================================
// Packet helpers
// =============================================================




static void sendResponse(TResponseType resp, uint32_t p0, uint32_t p1, uint32_t p2) {
    TPacket pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.packetType = PACKET_TYPE_RESPONSE;
    pkt.command    = resp;
    pkt.params[0]  = p0;
    pkt.params[1]  = p1;
    pkt.params[2]  = p2;
    sendFrame(&pkt);
}




static void sendStatus(TState state) {
    sendResponse(RESP_STATUS, (uint32_t)state, 0, 0);
}




// =============================================================
// E-Stop state machine
// =============================================================




#define ESTOP_PIN 3
#define DEBOUNCE_TIME 30




volatile TState  buttonState = STATE_RUNNING;
volatile bool    stateChanged = false;
volatile uint8_t was_running = 0;




ISR(INT3_vect) {
    static unsigned long lastTime = 0;
    unsigned long now = millis();
    if (now - lastTime < DEBOUNCE_TIME) return;
    lastTime = now;




    bool pressed = (PIND & ESTOP_PIN);




    if (buttonState == STATE_RUNNING && pressed) {
        buttonState  = STATE_STOPPED;
        stateChanged = true;
        was_running  = 1;
        move(0, MSTOP);
    } else if (was_running == 1 && buttonState == STATE_STOPPED && !pressed) {
        was_running = 2;
    } else if (was_running == 2 && buttonState == STATE_STOPPED && !pressed) {
        buttonState  = STATE_RUNNING;
        stateChanged = true;
        was_running  = 0;
    }
}




// =============================================================
// Color sensor (TCS3200)
// =============================================================




#define S0 (1 << 0)
#define S1 (1 << 1)
#define S2 (1 << 2)
#define S3 (1 << 3)
#define S_OUT (1 << 2)




#define LDR_READINGS 5




volatile uint32_t rising_edge_count   = 0;
volatile uint8_t  current_colour_stage = 0;
volatile uint32_t R_channel_value     = 0;
volatile uint32_t G_channel_value     = 0;
volatile uint32_t B_channel_value     = 0;
volatile uint8_t  reading_count       = 0;




static void readColourChannels() {
    current_colour_stage = 0;
    PORTA |=  S0;
    PORTA &= ~S1;

    R_channel_value = 0;
    G_channel_value = 0;
    B_channel_value = 0;

    TIMSK0 |= (1 << OCIE0A);
}




void enableChannel(uint8_t RGB) {
    switch (RGB) {
        case 0:
            PORTA &= ~(S2|S3);
            break;
        case 1:
            PORTA |= (S2|S3);
            break;
        case 2:
            PORTA &= ~S2;
            PORTA |= S3;
            break;
    }
}




ISR(TIMER0_COMPA_vect) {
    switch (current_colour_stage) {
        case 0:
            rising_edge_count = 0;
            enableChannel(0);
            reading_count = 0;
            current_colour_stage++;
            break;




        case 1:
            R_channel_value += rising_edge_count;
            rising_edge_count = 0;
            enableChannel(1);
            reading_count++;
            if (reading_count >= LDR_READINGS) {
                current_colour_stage++;
                reading_count = 0;
            }
            break;




        case 2:
            G_channel_value += rising_edge_count;
            rising_edge_count = 0;
            enableChannel(2);
            reading_count++;
            if (reading_count >= LDR_READINGS) {
                current_colour_stage++;
                reading_count = 0;
            }
            break;




        case 3:
            B_channel_value += rising_edge_count;
            rising_edge_count = 0;
            reading_count++;
            if (reading_count >= LDR_READINGS) {
                reading_count        = 0;
                current_colour_stage = 0;
                sendResponse(RESP_COLOUR_DATA,
                             R_channel_value * 10,
                             G_channel_value * 10,
                             B_channel_value * 10);
                TIMSK0 &= ~(1 << OCIE0A);
            }
            break;
    }
}




ISR(INT2_vect) {rising_edge_count++;}




// =============================================================
// Command handler
// =============================================================




static void handleCommand(const TPacket *cmd) {
    if (cmd->packetType != PACKET_TYPE_COMMAND) return;




    switch (cmd->command) {




        case COMMAND_ESTOP:
            cli();
            buttonState  = 1 - buttonState;
            stateChanged = true;
            sei();
            if (buttonState == STATE_STOPPED) move(0, MSTOP);
            break;




        case COMMAND_GET_COLOUR:
            readColourChannels();
            break;




        case COMMAND_FORWARD:
            if (buttonState == STATE_RUNNING) move(motorSpeed, GO);
            sendResponse(RESP_OK, 0, 0, 0);
            break;




        case COMMAND_BACKWARD:
            if (buttonState == STATE_RUNNING) move(motorSpeed, BACK);
            sendResponse(RESP_OK, 0, 0, 0);
            break;




        case COMMAND_TURN_LEFT:
            if (buttonState == STATE_RUNNING) move(motorSpeed, CCW);
            sendResponse(RESP_OK, 0, 0, 0);
            break;




        case COMMAND_TURN_RIGHT:
            if (buttonState == STATE_RUNNING) move(motorSpeed, CW);
            sendResponse(RESP_OK, 0, 0, 0);
            break;




        case COMMAND_STOP_MOTORS:
            move(0, MSTOP);
            sendResponse(RESP_OK, 0, 0, 0);
            break;




        case COMMAND_SET_SPEED:
            motorSpeed = (uint8_t)(cmd->params[0] & 0xFF);
             motorFL.setSpeed(motorSpeed);
             motorFR.setSpeed(motorSpeed);
             motorBL.setSpeed(motorSpeed);
             motorBR.setSpeed(motorSpeed);
            sendResponse(RESP_OK, motorSpeed, 0, 0);
            break;




        case COMMAND_CLAW_BASE: {
            int angle = (int)cmd->params[0];
            if (angle >= BASE_LOWER_LIMIT && angle <= BASE_UPPER_LIMIT)
                B_target = degToTicks(angle);
            sendResponse(RESP_OK, (uint32_t)angle, 0, 0);
            break;
        }




        case COMMAND_CLAW_SHOULDER: {
            int angle = (int)cmd->params[0];
            if (angle >= SHOULDER_LOWER_LIMIT && angle <= SHOULDER_UPPER_LIMIT)
                S_target = degToTicks(angle);
            sendResponse(RESP_OK, (uint32_t)angle, 0, 0);
            break;
        }




        case COMMAND_CLAW_ELBOW: {
            int angle = (int)cmd->params[0];
            if (angle >= ELBOW_LOWER_LIMIT && angle <= ELBOW_UPPER_LIMIT)
                E_target = degToTicks(angle);
            sendResponse(RESP_OK, (uint32_t)angle, 0, 0);
            break;
        }




        case COMMAND_CLAW_GRIPPER: {
            int angle = (int)cmd->params[0];
            if (angle >= GRIPPER_LOWER_LIMIT && angle <= GRIPPER_UPPER_LIMIT)
                G_target = degToTicks(angle);
            sendResponse(RESP_OK, (uint32_t)angle, 0, 0);
            break;
        }




        case COMMAND_CLAW_HOME:
            B_target = STARTTICKS;
            S_target = STARTTICKS;
            E_target = STARTTICKS;
            G_target = STARTTICKS;
            clawTPP  = START_TPP;
            sendResponse(RESP_OK, 0, 0, 0);
            break;




        case COMMAND_CLAW_SPEED: {
            unsigned int mpd = (unsigned int)cmd->params[0];
            if (mpd > 0) clawTPP = mpdToTPP(mpd);
            sendResponse(RESP_OK, clawTPP, 0, 0);
            break;
        }
    }
}




// =============================================================
// Arduino setup() and loop()
// =============================================================




void setup() {
    //motorFL.setSpeed(200);
    //motorFL.run(FORWARD);
    //delay(3000);
    // ... rest of setup




#if USE_BAREMETAL_SERIAL
    usartInit(103);
#else
    Serial.begin(9600);
#endif




    // E-Stop button interrupt (PD3 / INT3)
    DDRD  &= ~ESTOP_PIN;
    PORTD |= ESTOP_PIN; //set high for pullup
    EICRA |=  (1 << ISC30);
    EICRA &= ~(1 << ISC31);
    delay(100);
    buttonState = STATE_RUNNING;
    EIMSK |=  (1 << INT3);




    // Rising-edge counter fkor colour sensor (PD2 / INT2)
    DDRD &= ~S_OUT;    
    EICRA |= (1 << ISC21) | (1 << ISC20);
    EIMSK |= (1 << INT2);
   
    // Colour-sensor GPIO
    DDRA |= (S0|S1|S2|S3);
    PORTA &= ~(S0|S1|S2|S3);
    //Timer0: colour sensor window, CTC, 100 ms
    //prescaler=256, OCR3A=6249
    TCCR0A = 0;
    TCCR0B = (1 << WGM02) | (1 << CS02);
    OCR0A  = 6249;
    TCNT0  = 0;
    TIMSK0 = 0;




    // Claw servo GPIO
   DDRC  |=  (BASE_PIN | SHOULDER_PIN | ELBOW_PIN | GRIPPER_PIN);
   PORTC &= ~(BASE_PIN | SHOULDER_PIN | ELBOW_PIN | GRIPPER_PIN);
    // Timer5: claw servo PWM, CTC, 20 ms period
    //prescaler=8, TOP=39999
    TCCR5A = 0b0;
    TIMSK5 = 0b0;
    TCNT5  = 0;
    OCR5A  = TOP_CHECKPOINT;
    OCR5B  = B_CHECKPOINT;
    TIMSK5 = (1 << OCIE5A) | (1 << OCIE5B);
    TCCR5B = (1 << WGM52) | (1 << CS51);




    sei();




//  motorFL.setSpeed(200);// testing
//  motorFL.run(FORWARD); // testing
//  move(200, GO); // testing
}




void loop() {
    // --- 1. Report any E-Stop state change to the Pi ---
    if (stateChanged) {
        cli();
        TState state = buttonState;
        stateChanged = false;
        sei();
        sendStatus(state);
    }




    // --- 2. Process incoming commands from the Pi ---
    TPacket incoming;
    if (receiveFrame(&incoming)) {
        handleCommand(&incoming);
    }
}





