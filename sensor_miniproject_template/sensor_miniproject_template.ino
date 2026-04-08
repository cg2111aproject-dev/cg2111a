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
#define SHOULDER_LOWER_LIMIT  35 ///103 //35
#define SHOULDER_UPPER_LIMIT  153
#define ELBOW_LOWER_LIMIT     51 ///48 ///90 //110
#define ELBOW_UPPER_LIMIT     142 //138 //120
#define GRIPPER_LOWER_LIMIT   108 ///105 //(90 too wide)
#define GRIPPER_UPPER_LIMIT   125 ///120 ///130

//REMEMBER TO UPDATE THESE WHEN CHANGING ANYTHING ABOVE
#define BASE_LOWER_LIMIT_TICKS      1000
#define BASE_UPPER_LIMIT_TICKS      5000
#define SHOULDER_LOWER_LIMIT_TICKS  1778 ///3289 //1778
#define SHOULDER_UPPER_LIMIT_TICKS  4400 //4489
#define ELBOW_LOWER_LIMIT_TICKS     2133 ///2067 ///3000 //3444
#define ELBOW_UPPER_LIMIT_TICKS     4156 //4067 //3667
#define GRIPPER_LOWER_LIMIT_TICKS   3400 ///3333
#define GRIPPER_UPPER_LIMIT_TICKS   3778 ///3667 ///3889

#define BASE_PIN     (1 << 0)
#define SHOULDER_PIN (1 << 1)
#define ELBOW_PIN    (1 << 2)
#define GRIPPER_PIN  (1 << 3)


#define B_CHECKPOINT 0
#define S_CHECKPOINT 10000
#define E_CHECKPOINT 20000
#define G_CHECKPOINT 30000
#define TOP_CHECKPOINT 39999


#define startTPP 20
#define TPP_STEP 2
#define TPP_MAX 40
#define TPP_MIN 6

#define startticks 3444
#define S_startticks 1778 //1778 //3720
#define E_startticks 2978 //3196 //3256
#define TICKS_SP1 3420
#define TICKS_SP2 3576 //3506
#define TICKS_SP3 3732 //3594
#define TICKS_SP4 3888 //3814
#define TICKS_SP5 4044 //3968
#define TICKS_SP6 4200 //4111

//CLAW PRESETS

int stagecount = 0;

volatile unsigned int ticksperperiod = startTPP;
unsigned int B_currticks = startticks, S_currticks = S_startticks, E_currticks = E_startticks, G_currticks = startticks;
unsigned int B_target = startticks, S_target = S_startticks, E_target = E_startticks, G_target = startticks;




uint32_t MPD_to_TPP(uint32_t msperdeg){
  return round(444.44444 / msperdeg);
}


uint32_t degree_to_ticks(uint32_t deg){
  return round(deg * 22.22222) + 1000;
}

void reportAngle(){
  sendResponse(3, S_currticks, E_currticks, G_currticks);
}



ISR(TIMER5_COMPB_vect)
{
  switch (stagecount) {


    case 0:
      PORTC |= BASE_PIN;
      OCR5B += B_currticks;
      break;


    case 1:
      PORTC &= ~BASE_PIN;
      OCR5B = S_CHECKPOINT;
      break;


    case 2:
      PORTC |= SHOULDER_PIN;
      OCR5B += S_currticks;
      break;


    case 3:
      PORTC &= ~SHOULDER_PIN;
      OCR5B = E_CHECKPOINT;
      break;


    case 4:
      PORTC |= ELBOW_PIN;
      OCR5B += E_currticks;
      break;


    case 5:
      PORTC &= ~ELBOW_PIN;
      OCR5B = G_CHECKPOINT;
      break;


    case 6:
      PORTC |= GRIPPER_PIN;
      OCR5B += G_currticks;
      break;


    case 7:
      PORTC &= ~GRIPPER_PIN;
      OCR5B = B_CHECKPOINT;
      stagecount = -1;
      break;


  }


  stagecount++;
}


// Timer5 Compare A ISR — claw servo lerp tick every 20 ms
ISR(TIMER5_COMPA_vect) {
    //now_20ms++;
    lerp_ticks();
}


void lerp_ticks() {
//increment / decrement and CLAMP


  if (B_currticks < B_target) {

    if (B_currticks + ticksperperiod > B_target) B_currticks = B_target;
    else B_currticks += ticksperperiod;

  } else {

    if (B_currticks < ticksperperiod + B_target) B_currticks = B_target;
    else B_currticks -= ticksperperiod;

  }

  if (S_currticks < S_target) {

    if (S_currticks + ticksperperiod > S_target) S_currticks = S_target;
    else S_currticks += ticksperperiod;

  } else {

    if (S_currticks < ticksperperiod + S_target) S_currticks = S_target;
    else S_currticks -= ticksperperiod;

  }

  if (E_currticks < E_target) {

    if (E_currticks + ticksperperiod > E_target) E_currticks = E_target;
    else E_currticks += ticksperperiod;

  } else {

    if (E_currticks < ticksperperiod + E_target) E_currticks = E_target;
    else E_currticks -= ticksperperiod;

  }

  if (G_currticks < G_target) {

    if (G_currticks + ticksperperiod > G_target) G_currticks = G_target;
    else G_currticks += ticksperperiod;

  } else {

    if (G_currticks < ticksperperiod + G_target) G_currticks = G_target;
    else G_currticks -= ticksperperiod;

  }

}


// =============================================================
// E-Stop state machine
// =============================================================

#define ESTOP_PIN (1 << 3)
#define DEBOUNCE_TIME 100


volatile TState  buttonState = STATE_RUNNING;
volatile bool    stateChanged = false;
volatile uint8_t was_running = 0;
//volatile uint32_t now_20ms = 0;

// IGNORE to debounce, we increment now_20ms using TIMER5, which already interrupts every 20ms for the claw
// IGNORE now_20ms means "how many 20ms have passed"


ISR(INT3_vect) {
    static unsigned long lastTime = 0;
    unsigned long now = millis();
    //sendResponse(3, 'p', now, lastTime);

    if (now - lastTime < DEBOUNCE_TIME) return;
    lastTime = now;  
    
    bool pressed = (PIND & ESTOP_PIN);
    pressed = !pressed;
    //DEBUG
    //if (pressed) sendResponse(3, 1, PIND, ESTOP_PIN);
    //else sendResponse(3, 0, PIND, ESTOP_PIN);

    if (buttonState == STATE_RUNNING && pressed) {
        buttonState  = STATE_STOPPED;
        stateChanged = true;
        was_running  = 1;

        //HALT motors and claw and colour sensor
        move(0, MSTOP);
        B_target = B_currticks;
        S_target = S_currticks;
        E_target = E_currticks;
        G_target = G_currticks;
	TIMSK2 &= ~(1 << OCIE2A); //off timer2 interrupt (colour sensor)
	
	//NOTE: LIDAR WILL BE STOPPED BY RPI, RPI KNOWS THIS AS WE sendStatus in loop()


    } else if (was_running == 1 && buttonState == STATE_STOPPED && !pressed) {
        was_running = 2;
    } else if (was_running == 2 && buttonState == STATE_STOPPED && !pressed) {
        buttonState  = STATE_RUNNING;
        stateChanged = true;
        was_running  = 0;
    }
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
volatile uint8_t count100ms = 0;
//volatile bool enablecoloursensor = false;




//OBSOLETE
static void readColourChannels() {
    current_colour_stage = 0;
    PORTA |=  S0;
    PORTA &= ~S1;


    R_channel_value = 0;
    G_channel_value = 0;
    B_channel_value = 0;


    TIMSK2 |= (1 << OCIE2A);
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








ISR(TIMER2_COMPA_vect) {
    count100ms++;
    if (count100ms >= 10) {
        count100ms = 0;
        TIMER2_100MS_TRIGGER();
    }
}


void TIMER2_100MS_TRIGGER() {

    switch (current_colour_stage) {

        case 0:
	    R_channel_value = 0;
            G_channel_value = 0;
            B_channel_value = 0;

            rising_edge_count = 0;
            enableChannel(0);
            reading_count = 0;            
            current_colour_stage++;
            break;

        case 1:
            R_channel_value += rising_edge_count;
            rising_edge_count = 0;
            reading_count++;
            if (reading_count >= LDR_READINGS) {
                current_colour_stage++;
                reading_count = 0;
                enableChannel(1);
            }
            break;

        case 2:
            G_channel_value += rising_edge_count;
            rising_edge_count = 0;
            reading_count++;
            if (reading_count >= LDR_READINGS) {
                current_colour_stage++;
                reading_count = 0;
                enableChannel(2);
            }
            break;

        case 3:
            B_channel_value += rising_edge_count;
            rising_edge_count = 0;
            reading_count++;
            if (reading_count >= LDR_READINGS) {
                reading_count = 0;
                current_colour_stage = 0;
                rising_edge_count = 0;
                sendResponse(RESP_COLOUR_DATA, //arbitrary scaling by 10
                            R_channel_value * 10 / LDR_READINGS,
                            G_channel_value * 10 / LDR_READINGS,
                            B_channel_value * 10 / LDR_READINGS
                );
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

	    was_running  = 2; //SET THIS SO THE PHYSICAL ESTOP CAN UNSTOP

	    //HALT MOTOR, CLAW, COLOUR SENSOR
            if (buttonState == STATE_STOPPED) {
	        move(0, MSTOP);
	        B_target = B_currticks;
		S_target = S_currticks;
		E_target = E_currticks;
		G_target = G_currticks;
		TIMSK2 &= ~(1 << OCIE2A); //off timer2 interrupt
	    }
            break;




        case COMMAND_GET_COLOUR:

            //NEW START ---------------------
            count100ms = 9;
            current_colour_stage = 0;
            R_channel_value = 0;
            G_channel_value = 0;
            B_channel_value = 0;

            TIMSK2 |= (1 << OCIE2A); //on timer2 interrupt
            
            //NEW END -----------------------

            //readColourChannels(); //OLD

            break;

	case COMMAND_STOP_COLOUR:
		TIMSK2 &= ~(1 << OCIE2A); //off timer2 interrupt
		break;


        case COMMAND_FORWARD:
            if (buttonState == STATE_RUNNING) move(motorSpeed, GO);
            //sendResponse(RESP_OK, 0, 0, 0);
            break;


        case COMMAND_BACKWARD:
            if (buttonState == STATE_RUNNING) move(motorSpeed, BACK);
            //sendResponse(RESP_OK, 0, 0, 0);
            break;


        case COMMAND_TURN_LEFT:
            if (buttonState == STATE_RUNNING) move(motorSpeed, CCW);
            //sendResponse(RESP_OK, 0, 0, 0);
            break;


        case COMMAND_TURN_RIGHT:
            if (buttonState == STATE_RUNNING) move(motorSpeed, CW);
            //sendResponse(RESP_OK, 0, 0, 0);
            break;


        case COMMAND_STOP_MOTORS:
            move(0, MSTOP);
            //sendResponse(RESP_OK, 0, 0, 0);
            break;


        case COMMAND_SET_SPEED:
            motorSpeed = (uint8_t)(cmd->params[0] & 0xFF);
             motorFL.setSpeed(motorSpeed);
             motorFR.setSpeed(motorSpeed);
             motorBL.setSpeed(motorSpeed);
             motorBR.setSpeed(motorSpeed);
            sendResponse(RESP_OK, motorSpeed, 0, 0);
            break;

//----------

       case COMMAND_INCR_CLAW_BASE:
            B_target = BASE_UPPER_LIMIT_TICKS;
            break;

       case COMMAND_DECR_CLAW_BASE:
            B_target = BASE_LOWER_LIMIT_TICKS;
            break;

       case COMMAND_INCR_CLAW_SHOULDER:
            S_target = SHOULDER_UPPER_LIMIT_TICKS;
            break;

       case COMMAND_DECR_CLAW_SHOULDER:
            S_target = SHOULDER_LOWER_LIMIT_TICKS;
            break;

       case COMMAND_INCR_CLAW_ELBOW:
            E_target = ELBOW_UPPER_LIMIT_TICKS;
            break;

       case COMMAND_DECR_CLAW_ELBOW:
            E_target = ELBOW_LOWER_LIMIT_TICKS;
            break;

       case COMMAND_OPEN_GRIPPER:
            //clawmode = 0; //stop other movement
            //sendResponse(3, 'o', 'p', clawmode);
            G_target = GRIPPER_LOWER_LIMIT_TICKS;
            G_currticks = GRIPPER_LOWER_LIMIT_TICKS;
            //motorFL.run(FORWARD);
            break;

       case COMMAND_CLOSE_GRIPPER:
            //clawmode = 0; //stop other movement
            //sendResponse(3, 'c', 'l', clawmode);
            G_target = GRIPPER_UPPER_LIMIT_TICKS;
            G_currticks = GRIPPER_UPPER_LIMIT_TICKS;
            break;

       case COMMAND_STOP_CLAW:
            B_target = B_currticks;
	        S_target = S_currticks;
            E_target = E_currticks;
            G_target = G_currticks;
            //sendResponse(3, S_target, E_target, G_target);
            break;       
            
       case COMMAND_INCR_CLAW_SPEED:
	    reportAngle();
	    if (ticksperperiod + TPP_STEP > TPP_MAX) ticksperperiod = TPP_MAX;
	    else ticksperperiod += TPP_STEP;
	    break;

       case COMMAND_DECR_CLAW_SPEED:
	    if (ticksperperiod < TPP_STEP + TPP_MIN) ticksperperiod = TPP_MIN;
	    else ticksperperiod -= TPP_STEP;
	    break;

       case COMMAND_CLAW_HOME:
            B_target = startticks;
            S_target = S_startticks;
            E_target = E_startticks;
            //G_target = startticks;
            ticksperperiod = startTPP;
            //sendResponse(RESP_OK, 0, 0, 0);
            break;

       case COMMAND_CLAW_P1:
            S_target = TICKS_SP1;
            break;

       case COMMAND_CLAW_P2:
            S_target = TICKS_SP2;
            break;

       case COMMAND_CLAW_P3:
            S_target = TICKS_SP3;
            break;

       case COMMAND_CLAW_P4:
            S_target = TICKS_SP4;
            break;

       case COMMAND_CLAW_P5:
            S_target = TICKS_SP5;
            break;

       case COMMAND_CLAW_P6:
            S_target = TICKS_SP6;
            break;

//----------

        case COMMAND_CLAW_BASE: {
            uint32_t angle = cmd->params[0];
            if (angle >= BASE_LOWER_LIMIT && angle <= BASE_UPPER_LIMIT)
                B_target = degree_to_ticks(angle);
            sendResponse(RESP_OK, (uint32_t)angle, 0, 0);
           

        //sendResponse(3, (uint32_t)B_target, 0, 0);
        break;
        }

        case COMMAND_CLAW_SHOULDER: {
            uint32_t angle = cmd->params[0];
            if (angle >= SHOULDER_LOWER_LIMIT && angle <= SHOULDER_UPPER_LIMIT)
                S_target = degree_to_ticks(angle);
            sendResponse(RESP_OK, (uint32_t)angle, 0, 0);
            break;
        }


        case COMMAND_CLAW_ELBOW: {
            uint32_t angle = cmd->params[0];
            if (angle >= ELBOW_LOWER_LIMIT && angle <= ELBOW_UPPER_LIMIT)
                E_target = degree_to_ticks(angle);
            sendResponse(RESP_OK, (uint32_t)angle, 0, 0);
            break;
        }

        case COMMAND_CLAW_GRIPPER: {
            uint32_t angle = cmd->params[0];
            if (angle >= GRIPPER_LOWER_LIMIT && angle <= GRIPPER_UPPER_LIMIT)
                G_target = degree_to_ticks(angle);
            sendResponse(RESP_OK, (uint32_t)angle, 0, 0);
            break;
        }


        case COMMAND_CLAW_SPEED: {
            uint32_t mpd = cmd->params[0];
            if (mpd > 0) ticksperperiod = MPD_to_TPP(mpd);
            sendResponse(RESP_OK, ticksperperiod, 0, 0);
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


    //cli(); //?




    // E-Stop button interrupt (PD3 / INT3)
    DDRD  &= ~ESTOP_PIN;
    PORTD |= ESTOP_PIN; //set high for pullup
    EICRA |=  (1 << ISC30);
    EICRA &= ~(1 << ISC31);
    delay(100);
    buttonState = STATE_RUNNING;
    EIMSK |=  (1 << INT3);








    // Rising-edge counter for colour sensor (PD2 / INT2)
    DDRD &= ~S_OUT;    
    EICRA |= (1 << ISC21) | (1 << ISC20);
    EIMSK |= (1 << INT2);
   
    // Colour-sensor GPIO
    DDRA |= (S0|S1|S2|S3);
    PORTA &= ~(S0|S1|S2|S3);
    //Timer2: colour sensor window, CTC, 10 x 10ms = 100 ms
    //prescaler = 1024
    TCCR2A = 0;
    TCCR2B = (1 << WGM22) | (1 << CS22) | (1 << CS21) | (1 << CS20);
    OCR2A  = 155;
    TCNT2  = 0;
    TIMSK2 = 0;

    PORTA |=  S0;
    PORTA &= ~S1;





    DDRC |= BASE_PIN | SHOULDER_PIN | ELBOW_PIN | GRIPPER_PIN;

    TCCR5A = 0b0; //WGM11 WGM10
    TIMSK5 |= 0b0110; //enable A and B interrupts
    TCNT5 = 0;


    OCR5A = TOP_CHECKPOINT; //20ms period
    OCR5B = 0;


    TCCR5B = 0b00001010; //WGM13 = 0, WGM12 = 1, prescalar = 8


 








    sei();
    PORTC &= ~(BASE_PIN | SHOULDER_PIN | ELBOW_PIN | GRIPPER_PIN); //set all pins to LOW (CLAW)








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













