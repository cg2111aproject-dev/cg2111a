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
#define BUTTON_PIN 2
#define S0 0
#define S1 1
#define S2 2
#define S3 3
#define S_OUT 4

#define LDR_READINGS 5
#define DEBOUNCE_TIME 30
// =============================================================
// Packet helpers (pre-implemented for you)
// =============================================================

/*
 * Build a zero-initialised TPacket, set packetType = PACKET_TYPE_RESPONSE,
 * command = resp, and params[0] = param.  Then call sendFrame().
 */
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

/*
 * Send a RESP_STATUS packet with the current state in params[0].
 */
static void sendStatus(TState state) {
    sendResponse(RESP_STATUS, (uint32_t)state, 0, 0);
}

// =============================================================
// E-Stop state machine
// =============================================================

volatile TState buttonState = STATE_RUNNING;
volatile bool  stateChanged = false;
volatile uint8_t was_running = 0; //count variable for debouncing the button in the ISR, we want to see two consecutive presses before we consider the button to be in the STOPPED state, and we want to reset the count if we see a release in between or if we are already in the STOPPED state and see another press

/*
 * TODO (Activity 1): Implement the E-Stop ISR.
 *
 * Fire on any logical change on the button pin.
 * State machine (see handout diagram):
 *   RUNNING + press (pin HIGH)  ->  STOPPED, set stateChanged = true
 *   STOPPED + release (pin LOW) ->  RUNNING, set stateChanged = true
 *
 * Debounce the button.  You will also need to enable this interrupt
 * in setup() -- check the ATMega2560 datasheet for the correct
 * registers for your chosen pin.
 */

// ISR(INT5_vect) {
//     static unsigned long lastTime = 0;
//     unsigned long now = millis();
//     if (now - lastTime < 300) return;  // increase to 300ms
//     lastTime = now;

//     bool pressed = (PINE & (1 << PINE5));

//     if (buttonState == STATE_RUNNING && pressed) {
//         buttonState = STATE_STOPPED;
//         stateChanged = true;
//         was_running = 1;
//     } else if (was_running == 1 && buttonState == STATE_STOPPED && !pressed) {
//         was_running = 0;
//     } else if (was_running == 0 && buttonState == STATE_STOPPED && pressed) {
//         was_running = 2;
//     } else if (was_running == 2 && buttonState == STATE_STOPPED && !pressed) {
//         buttonState = STATE_RUNNING;
//         stateChanged = true;
//         was_running = 0;
//     }
// }
// ISR(INT5_vect) {
//     static unsigned long lastTime = 0;
//     unsigned long now = millis();
//     if (now - lastTime < 50) return;
//     lastTime = now;

//     bool pressed = !(PINE & (1 << PINE5));  // LOW = pressed with pull-up

//     if (buttonState == STATE_RUNNING && pressed) {
//         buttonState = STATE_STOPPED;
//         stateChanged = true;
//     } else if (buttonState == STATE_STOPPED && !pressed) {
//         buttonState = STATE_RUNNING;
//         stateChanged = true;
//     }
// }
ISR(INT5_vect) {
    static unsigned long lastTime = 0;
    unsigned long now = millis();
    if (now - lastTime < DEBOUNCE_TIME) return;
    lastTime = now;

    bool pressed = (PINE & (1 << PINE5));  // LOW = pressed with pull-up

    if (buttonState == STATE_RUNNING && pressed) {
        // First press: go to STOPPED
        buttonState = STATE_STOPPED;
        stateChanged = true;
        was_running = 1;

    } else if (was_running == 1 && buttonState == STATE_STOPPED && !pressed) {
        // First release after being stopped: waiting for second press
        was_running = 2;

    } else if (was_running == 2 && buttonState == STATE_STOPPED && !pressed) {
        // Second press: go back to RUNNING
        buttonState = STATE_RUNNING;
        stateChanged = true;
        was_running = 0;
    }
}


// =============================================================
// Color sensor (TCS3200)
// =============================================================
volatile uint32_t rising_edge_count = 0;
volatile uint8_t current_colour_stage = 0;
volatile uint32_t R_channel_value = 0;
volatile uint32_t G_channel_value = 0;
volatile uint32_t B_channel_value = 0;
volatile uint8_t reading_count = 0;

/*
 * TODO (Activity 2): Implement the color sensor.
 *
 * Wire the TCS3200 to the Arduino Mega and configure the output pins
 * (S0, S1, S2, S3) and the frequency output pin.
 *
 * Use 20% output frequency scaling (S0=HIGH, S1=LOW).  This is the
 * required standardised setting; it gives a convenient measurement range and
 * ensures all implementations report the same physical quantity.
 *
 * Use a timer to count rising edges on the sensor output over a fixed
 * window (e.g. 100 ms) for each color channel (red, green, blue).
 * Convert the edge count to hertz before sending:
 *   frequency_Hz = edge_count / measurement_window_s
 * 
 *
 * Implement a function that measures all three channels and stores the
 * frequency in Hz in three variables.
 *
 * Define your own command and response types in packets.h (and matching
 * constants in pi_sensor.py), then handle the command in handleCommand()
 * and send back the channel frequencies (in Hz) in a response packet.
 *
 * Example skeleton:
 *
 *   static void readColorChannels(uint32_t *r, uint32_t *g, uint32_t *b) {
 *       // Set S2/S3 for each channel, measure edge count, multiply by 10
 *       *r = measureChannel(0, 0) * 10;  // red,   in Hz
 *       *g = measureChannel(1, 1) * 10;  // green, in Hz
 *       *b = measureChannel(0, 1) * 10;  // blue,  in Hz
 *   }
 */
static void readColourChannels() {
    // Set S0 and S1 for 20% frequency scaling
    // sendResponse(RESP_COLOUR_DATA, R_channel_value * 10 / LDR_READINGS, G_channel_value * 10 / LDR_READINGS, B_channel_value * 10 / LDR_READINGS); //send the response with the channel frequencies in Hz (remember to multiply by 10 for 100 ms window)
    current_colour_stage = 0;
    PORTA |= (1 << S0);  // S0 HIGH
    PORTA &= ~(1 << S1); // S1 LOW

    //RESET CHANNEL VALUES
    R_channel_value = 0;
    G_channel_value = 0;
    B_channel_value = 0;

    // Start with red channel (S2=LOW, S3=LOW)
    TIMSK1 |= (0b10); //enable timer 1 OCR1A interrupt to start measuring

    // The rest of the measurement is handled in the TIMER1_COMPA_vect ISR,
    // which updates the channel values and moves through the stages.
}

void enableChannel(uint8_t RGB) {
    //0 - red, 1 - green, 2 - blue
    
    switch (RGB) {
        case 0:
            PORTA &= ~(0b00001100); //S2 = LOW, S3 = LOW
            break;

        case 1:
            PORTA |= (0b00001100); //S2 = HIGH, S3 = HIGH
            break;

        case 2:
            PORTA &= ~(0b00000100); //S2 = LOW
            PORTA |= (0b00001000); //S3 = HIGH
            break;
    }
}

ISR(TIMER1_COMPA_vect) {
    switch (current_colour_stage) {
        case 0:
            rising_edge_count = 0; //reset count as we are going to measure red now
            enableChannel(0);
	    reading_count = 0;
            current_colour_stage++;
        break;

        case 1:
            R_channel_value += rising_edge_count; //update red channel value (REPLACE THIS)
            rising_edge_count = 0; //reset count as we are going to measure green now
            enableChannel(1); //set both S2 AND S3 TO HIGH (GREEN)
	        reading_count++;

	        if (reading_count >= LDR_READINGS) {
		    current_colour_stage++;
		    reading_count = 0;
	        }
         break;

        case 2:
            G_channel_value += rising_edge_count; //update green channel value (REPLACE THIS)
            rising_edge_count = 0; //reset count as we are going to measure blue now
            enableChannel(2); //set both S2 to LOW, S3 TO HIGH (BLUE)
            reading_count++;
            if (reading_count >= LDR_READINGS) {
		    current_colour_stage++;
		    reading_count = 0;
            }
            break;

        case 3:
            B_channel_value += rising_edge_count; //update green channel value (REPLACE THIS)
            rising_edge_count = 0; //reset count
            reading_count++;
            if (reading_count >= LDR_READINGS) {
		    reading_count = 0;
            current_colour_stage = 0;
            sendResponse(RESP_COLOUR_DATA, R_channel_value * 10, G_channel_value * 10, B_channel_value * 10); //send the response with the channel frequencies in Hz (remember to multiply by 10 for 100 ms window)
            TIMSK1 &= ~(0b10); //disable timer 1 OCR1A interrupt
            }
            break;
    }

}

ISR(INT4_vect) {
    rising_edge_count++; //increment count on every rising edge from the sensor output
}

// =============================================================
// Command handler
// =============================================================

/*
 * Dispatch incoming commands from the Pi.
 *
 * COMMAND_ESTOP is pre-implemented: it sets the Arduino to STATE_STOPPED
 * and sends back RESP_OK followed by a RESP_STATUS update.
 *
 * TODO (Activity 2): add a case for your color sensor command.
 *   Call your color-reading function, then send a response packet with
 *   the channel frequencies in Hz.
 */
static void handleCommand(const TPacket *cmd) {
    if (cmd->packetType != PACKET_TYPE_COMMAND) return;

    switch (cmd->command) {
        case COMMAND_ESTOP:
            cli();
            buttonState  = 1 - buttonState;
            stateChanged = true;
            sei();
            /*{
                // The data field of a TPacket can carry a short debug string (up to
                // 31 characters).  pi_sensor.py prints it automatically for any packet
                // where data is non-empty, so you can use it to send debug messages
                // from the Arduino to the Pi terminal -- similar to Serial.print().
                TPacket pkt;
                memset(&pkt, 0, sizeof(pkt));
                pkt.packetType = PACKET_TYPE_RESPONSE;
                pkt.command    = RESP_OK;
                strncpy(pkt.data, "This is a debug message", sizeof(pkt.data) - 1);
                pkt.data[sizeof(pkt.data) - 1] = '\0';
                sendFrame(&pkt);
            }*/
            //sendStatus(STATE_STOPPED);
            break;

        case COMMAND_GET_COLOUR:
            readColourChannels();
            break;
        // TODO (Activity 2): add COMMAND_COLOR case here.
        //   Call your color-reading function (which returns Hz), then send a
        //   response packet with the three channel frequencies in Hz.
    }
}

// =============================================================
// Arduino setup() and loop()
// =============================================================

void setup() {
    // Initialise the serial link at 9600 baud.
    // Serial.begin() is used by default; usartInit() takes over once
    // USE_BAREMETAL_SERIAL is set to 1 in serial_driver.h.
#if USE_BAREMETAL_SERIAL
    usartInit(103);   // 9600 baud at 16 MHz
#else
    Serial.begin(9600);
#endif
    // TODO (Activity 1): configure the button pin and its external interrupt,
    // then call sei() to enable global interrupts.
    // 1. Set button as input
    DDRE &= ~(1 << DDE5);     // PE5 as input
    PORTE |= (1 << PORTE5);  //  pull-up
    EICRB |= (1 << ISC50);    // any logical change on INT5
    EICRB &= ~(1 << ISC51);
    EIMSK |= (1 << INT5);

    // 2. Configure External Interrupt Control Register A (EICRA)
    //configure INT4 for rising edge detection from the sensor output (ISC41=1, ISC40=1)
    EICRB |= (1 << ISC41) | (1 << ISC40);

    // 3. Enable INT0 in External Interrupt Mask Register (EIMSK)
    EIMSK |= (1 << INT4); //enable INT4 for counting rising edges from the sensor output
    DDRA |= (1 << S0);
    DDRA |= (1 << S1);
    DDRA |= (1 << S2);
    DDRA |= (1 << S3);
    // DDRD &= ~(1 << S_OUT);
    DDRE &= ~(1 << S_OUT); //configure sensor output pin as input
    PORTA = 0b0;
    TCCR1A = 0b0;
    TIMSK1 = 0b0; //disable timer1 interrupt first
    TCNT1 = 0;
    OCR1AH = 0b00011000; //for 256 prescalar and 100ms OCR1A = 6249
    OCR1AL = 0b01101001;
    TCCR1B = 0b00001100; //start timer1, it just wont trigger the interrupt
    
    sei();
    
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
