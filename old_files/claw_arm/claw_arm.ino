// #include <Servo.h>

#include <avr/io.h>
#include <avr/interrupt.h>


//SET THESE
// #define BASE_LOWER_LIMIT 
// #define BASE_UPPER_LIMIT 5000
// #define SHOULDER_LOWER_LIMIT 1000
// #define SHOULDER_UPPER_LIMIT 5000
// #define ELBOW_LOWER_LIMIT 1000
// #define ELBOW_UPPER_LIMIT 5000
// #define GRIPPER_LOWER_LIMIT 1000
// #define GRIPPER_UPPER_LIMIT 5000

#define BASE_LOWER_LIMIT 0 //increase angle to move anticlockwise
#define BASE_UPPER_LIMIT 180
#define SHOULDER_LOWER_LIMIT 50 //increase angle to go up
#define SHOULDER_UPPER_LIMIT 135
#define ELBOW_LOWER_LIMIT 105 //increase angle to move forward
#define ELBOW_UPPER_LIMIT 180
#define GRIPPER_LOWER_LIMIT 90 //increase angle to close
#define GRIPPER_UPPER_LIMIT 105

//base 180, elbow 180, 

#define B_CHECKPOINT 0
#define S_CHECKPOINT 10000
#define E_CHECKPOINT 20000
#define G_CHECKPOINT 30000
#define TOP_CHECKPOINT 39999

#define startticks 3000
#define startTPP 20;

int stagecount = 0;

unsigned int ticksperperiod = startTPP;
unsigned int B_currticks = startticks, S_currticks = startticks, E_currticks = startticks, G_currticks = startticks;
unsigned int B_target = startticks, S_target = startticks, E_target = startticks, G_target = startticks;

//arduino pins D4 to D7
#define BASE_PIN (1 << 0)
#define SHOULDER_PIN (1 << 1)
#define ELBOW_PIN (1 << 2)
#define GRIPPER_PIN (1 << 3)

void setup() {
  Serial.begin(115200);

  cli();
  DDRC |= BASE_PIN | SHOULDER_PIN | ELBOW_PIN | GRIPPER_PIN;

  TCCR5A = 0b0; //WGM11 WGM10
  TIMSK5 |= 0b0110; //enable A and B interrupts
  TCNT5 = 0;

  OCR5A = TOP_CHECKPOINT; //20ms period
  OCR5B = 0;

  TCCR5B = 0b00001010; //WGM13 = 0, WGM12 = 1, prescalar = 8
  sei();

  PORTC &= ~(BASE_PIN | SHOULDER_PIN | ELBOW_PIN | GRIPPER_PIN); //set all pins to LOW
  // PORTC |= BASE_PIN;
  // OCR1B = B_currticks;
}

ISR(TIMER5_COMPA_vect)
{
  lerp_ticks();
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



int parse3(const String *s) {
  if (!s) return -1;
  if (s->length() != 3) return -1;
  if (!isDigit(s->charAt(0)) || !isDigit(s->charAt(1)) || !isDigit(s->charAt(2))) return -1;
  return (s->charAt(0) - '0') * 100 + (s->charAt(1) - '0') * 10 + (s->charAt(2) - '0');
}





void loop() {
  if (!Serial.available()) return;

  // Reads a string with the command until newline
  String cmd = Serial.readStringUntil('\n');
  cmd.trim(); // Remove any extra whitespace
  if (!cmd.length()) return; // didn't read anything

  // Handle the home command
  if (cmd == "H") {
    Serial.println("Homing all servos...");
    homeAll();
    return;
  }

  // All subsequent commands need to have 4 characters
  if (cmd.length() != 4) {
    Serial.println("ERROR: Command is not 4 characters long");
    return;
  }

  // c is now the command character
  char c = cmd.charAt(0);
  // val is the numerical value of the argument
  int val = parse3(&cmd.substring(1));
  if (val < 0) { 
    Serial.println("ERROR: Argument not valid");
    return;
  }

  // Vddd sets velocity as ms per degree
  if (c == 'V') {
    Serial.print("Setting velocity to ");
    Serial.println(val);
    // msPerDeg = val;
    ticksperperiod = MPD_to_TPP(val);
    if (ticksperperiod == 0) ticksperperiod = 1;
    return;
  } else if (c == 'B') {
    Serial.print("Moving base to ");
    Serial.println(val);

    if (val < BASE_LOWER_LIMIT || val > BASE_UPPER_LIMIT) Serial.println("BASE degree out of bounds");
    else B_target = degree_to_ticks(val);

    return;
  } else if (c == 'S') {
    Serial.print("Moving shoulder to ");
    Serial.println(val);

    if (val < SHOULDER_LOWER_LIMIT || val > SHOULDER_UPPER_LIMIT) Serial.println("SHOULDER degree out of bounds");
    else S_target = degree_to_ticks(val);

    return;
  } else if (c == 'E') {
    Serial.print("Moving elbow to ");
    Serial.println(val);

    if (val < ELBOW_LOWER_LIMIT || val > ELBOW_UPPER_LIMIT) Serial.println("ELBOW degree out of bounds");
    else E_target = degree_to_ticks(val);

    return;
  } else if (c == 'G') {
    Serial.print("Moving gripper to ");
    Serial.println(val);

    if (val < GRIPPER_LOWER_LIMIT || val > GRIPPER_UPPER_LIMIT) Serial.println("GRIPPER degree out of bounds");
    else G_target = degree_to_ticks(val);

    return;
  } else {
    Serial.println("ERROR: Unknown command");
    return;
  }

  // lerp_ticks();
  // delay(20);

}

void lerp_ticks() {
//increment / decrement and CLAMP


  if (B_currticks < B_target) {
    B_currticks += ticksperperiod;
    if (B_currticks > B_target) B_currticks = B_target;
  } else {
    B_currticks -= ticksperperiod;
    if (B_currticks < B_target) B_currticks = B_target;
  }

  if (S_currticks < S_target) {
    S_currticks += ticksperperiod;
    if (S_currticks > S_target) S_currticks = S_target;
  } else {
    S_currticks -= ticksperperiod;
    if (S_currticks < S_target) S_currticks = S_target;
  }

  if (E_currticks < E_target) {
    E_currticks += ticksperperiod;
    if (E_currticks > E_target) E_currticks = E_target;
  } else {
    E_currticks -= ticksperperiod;
    if (E_currticks < E_target) E_currticks = E_target;
  }

  if (G_currticks < G_target) {
    G_currticks += ticksperperiod;
    if (G_currticks > G_target) G_currticks = G_target;
  } else {
    G_currticks -= ticksperperiod;
    if (G_currticks < G_target) G_currticks = G_target;
  }

}

void homeAll(){

  B_target = startticks;
  S_target = startticks;
  E_target = startticks;
  G_target = startticks;
  ticksperperiod = startTPP;

}

int MPD_to_TPP(unsigned int msperdeg){

  return round(444.44444 / msperdeg);

}

int degree_to_ticks(int deg){

  return round(deg * 22.22222) + 1000;

}
