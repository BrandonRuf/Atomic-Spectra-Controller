/*
 * For use in the Atomic Spectra experiment in McGill University physics course(s) PHYS-359/439.
 * Written by Brandon Ruffolo in 2023.
 */
#include "Vrekrer_scpi_parser.h"

#define SKETCH_VERSION "0.0.1"
#define BAUD 115200 // Baudrate  

/* Pin Definitions */
#define PIN_STEP              6  // Stepper motor step            
#define PIN_DIR               7  // Stepper motor direction
#define PIN_OFFSET            9  // Analog output pin to apply an offset bias to the PMT opamp
#define PIN_SWITCH_MAX        5  // Limit switch at the far end of the Monochromator
#define PIN_SWITCH_MIN        8  // Limit switch at the begining of the Monochromator
#define PIN_SWITCH_MAX_BACKUP 4  // Limit switch at the far end of the Monochromator, backup in case of failure of the main max limit switch
#define PIN_KNOB              A0
#define PIN_PMT               A1

/* Constants */
const uint16_t MICROSTEP_RATIO     = 400;   // Microstepping ratio (number of microsteps per revolution)
const uint16_t MAX_STEP            = 58868; // (VERIFIED EMPIRICALLY)
const uint16_t MIN_STEP            = 164;   // (VERIFIED EMPIRICALLY)
const uint16_t DEFAULT_STEP_DELAY  = 2000;  // Motor step pulse delay
const uint16_t HOMING_SETTLE_DELAY = 1000;  // Delay used between motor steps in the homing function
const uint16_t HOMING_SWITCH_DELAY = 500;
const uint8_t  PMT_DEFAULT_OFFSET  = 10;

/* Motor */
uint16_t motor_position  = 1; // $$(initially 1 to account for arduino reset?)$$
uint8_t  motor_direction = 0;

/* PMT */
uint8_t pmt_offset = PMT_DEFAULT_OFFSET;

/* Homing */
enum HOMING{NOT_DONE, COMPLETED, FAILED, RECAL};
enum HOMING home_status    = NOT_DONE;
const char *HOMING_NAMES[] = {"Not Done","Completed","Failed","Recalibrate"};

bool _debug = true;

SCPI_Parser Monochromator;

void setup()
{
  /* SCPI commands */
  Monochromator.RegisterCommand(F("*IDN?")    , &Identify);
  Monochromator.RegisterCommand(F("STATus?")  , &GetStatus);
  Monochromator.RegisterCommand(F("POSition?"), &GetPosition);
  Monochromator.RegisterCommand(F("POSition") , &SetPosition);
  Monochromator.RegisterCommand(F("HOME")     , &Home);
  Monochromator.RegisterCommand(F("DEBUG")    , &SetDebug);

  Monochromator.SetCommandTreeBase(F("STAGE"));
    Monochromator.RegisterCommand(F(":DIR?"), &GetMotorDir);
    Monochromator.RegisterCommand(F(":DIR") , &SetMotorDir);
    Monochromator.RegisterCommand(F(":STEP"), &StepMotor);
    Monochromator.SetCommandTreeBase(F("STAGE:LIMit"));
      Monochromator.RegisterCommand(F(":MAX?"), &GetMaxLimitState);
      Monochromator.RegisterCommand(F(":MIN?"), &GetMinLimitState);
  
  Monochromator.SetCommandTreeBase(F("PMT"));
    Monochromator.RegisterCommand(F(":VALue?") , &GetPMT);
    Monochromator.RegisterCommand(F(":OFFSET?"), &GetPMTOffset);
    Monochromator.RegisterCommand(F(":OFFSET") , &SetPMTOffset);


  pinMode(PIN_STEP,       OUTPUT); // Motor stepping pin
  pinMode(PIN_DIR,        OUTPUT); // Motor direction pin
  pinMode(PIN_OFFSET,     OUTPUT); // PMT I-to-V opamp offset bias voltage.
  pinMode(LED_BUILTIN,    OUTPUT); // LED for debugging
  pinMode(PIN_SWITCH_MAX, INPUT);  // Max limit switch monitor  
  pinMode(PIN_SWITCH_MIN, INPUT);  // Min limit switch monitor 
  
  analogWrite(PIN_OFFSET, pmt_offset);
  
  Serial.begin(BAUD);
}
void loop(){Monochromator.ProcessInput(Serial, "\n");}


void Identify(SCPI_C commands, SCPI_P parameters, Stream& interface) {
  interface.println(F("Ugrad Labs, Monochromator Controller, v" SKETCH_VERSION ", " __DATE__));
  // "<vendor>,<model>,<serial number>,<firmware>"
}

void GetStatus(SCPI_C commands, SCPI_P parameters, Stream& interface){
  interface.println("Homing: " +String(HOMING_NAMES[home_status]));
}

void Home(SCPI_C commands, SCPI_P parameters, Stream& interface){ /*
   * Homes the Monochromator.
   * This will run the monochromator toward one end until a limit switch is 
   * depressed, and will then return the stage to it's original position. 
   * 
   * Function GetStatus() can be called to see the result of the homing.
   * Following successful homing (GetStatus() == COMPLETED), the 
   * motor_position variable will contain the current position of the stage.
   */
  if(_debug){
    digitalWrite(LED_BUILTIN,HIGH);
    interface.println("Homing...");
  }

  motor_position = 0;                 // Reset motor position variable
  set_direction(LOW);                 // Set to increasing motor direction

  /* Step the motor and count steps until hitting the switch */
  while(1){ 
      if( motor_position > MAX_STEP){
        home_status = FAILED;        // Record homing failure
        interface.println("Homing Failed. Error code -1, see a technician for assistance."); 
        break;                       // Exit
      } 
      if( digitalRead(PIN_SWITCH_MAX) ) break; // Exit if switch was triggered (we're done stepping) 
      singleStep();                            // Step the motor
      delayMicroseconds(HOMING_SETTLE_DELAY);  // An additional delay to help microswitches settle
      
      if(Serial.available()){ /* Treat any incoming serial data as a signal to stop the move. */
        interface.println("Homing interrupted.");
        if(_debug) digitalWrite(LED_BUILTIN,LOW);
        return;                    
      }
  }
  set_direction(HIGH);                    // Reverse direction
  uint16_t displacement = motor_position; // Save total displacement to the limit switch
  motor_position = MAX_STEP;              // Set the motor position to its (now) known location  

  /* (Edge case) If the motor is one quarter rev from maximum position, advance it by 1 turn. */
  if(displacement <= 100) displacement = 400;  

  /**/
  delay(HOMING_SWITCH_DELAY);
  
  /* Bring the motor back to its original position */
  while(displacement){
    singleStep();
    displacement--;
  }
  
  if(_debug) digitalWrite(LED_BUILTIN,LOW);

  /* Update motor calibration status */
  if(home_status != FAILED) home_status = COMPLETED;
} 

void SetDebug(SCPI_C commands, SCPI_P parameters, Stream& interface){
  if (parameters.Size() > 0) {
      uint8_t setting = constrain(String(parameters[0]).toInt(), 0, 1);
      _debug = setting;
      } 
}

void GetPosition(SCPI_C commands, SCPI_P parameters, Stream& interface) {
  interface.println(String(motor_position, DEC));
}

void SetPosition(SCPI_C commands, SCPI_P parameters, Stream& interface) {
  uint16_t new_position;
  float x;
  
  if (parameters.Size() > 0) {
    x = constrain(String(parameters[0]).toFloat(), (float)MIN_STEP/MICROSTEP_RATIO, (float)MAX_STEP/MICROSTEP_RATIO );
    if(_debug) interface.println(String(x, DEC));
  }
  else{
    interface.println("No parameter supplied.");
    return;
  }

  new_position = (uint16_t)(x*MICROSTEP_RATIO);
  
  if(home_status != COMPLETED){
    interface.println("Homing not properly completed. Position cannot be set.");
    return;
  }
  if(new_position > MAX_STEP || new_position < MIN_STEP ){
    interface.println("Position is out of range.");
    return;  
  }
  if(new_position > motor_position) motor_direction      = LOW;     // Set to forward direction 
  else if(new_position < motor_position) motor_direction = HIGH;    // Set to reverse direction
  else                                                     return;  // Exit (we are already there)

  /* Step until we arrive at desired position */
  while(motor_position != new_position){ 
    
    /* Check limit switches (in case something is horribly wrong with the calibration!) */
    if (digitalRead(PIN_SWITCH_MAX) && motor_direction == LOW){  // MAX limit switch hit before expected.
      home_status = RECAL;                                       // Call for a recalibration of the stage
      interface.println("MAX limit switch hit before expected. Stage recalibration needed.");
      return; }
    if (digitalRead(PIN_SWITCH_MIN)&& motor_direction == HIGH){  // MIN limit switch hit before expected.
      home_status = RECAL;                                       // Call for a recalibration of the stage
      interface.println("MIN limit switch hit before expected. Stage recalibration needed.");
      return;}

    /* Treat any incoming serial data as a signal to stop the move. */
    if(Serial.available()){
      //while(Serial.available()) Serial.read(); // Flush the recieve buffer
      interface.println("Positioning stopped.");
      return;
    }
    
    singleStep();
  }
}

void GetPMT(SCPI_C commands, SCPI_P parameters, Stream& interface) {
  uint8_t pmt_val = analogRead(PIN_PMT);
  interface.println(String(pmt_val, DEC));
}

void SetPMTOffset(SCPI_C commands, SCPI_P parameters, Stream& interface) {
   if (parameters.Size() > 0) {
    pmt_offset = constrain(String(parameters[0]).toInt(), 0, 127);
    analogWrite(PIN_OFFSET, pmt_offset);
    if(_debug) interface.println(String(pmt_offset, DEC));
  }
}

void GetPMTOffset(SCPI_C commands, SCPI_P parameters, Stream& interface) {
  interface.println(String(pmt_offset, DEC));
}

void StepMotor(SCPI_C commands, SCPI_P parameters, Stream& interface) {
  uint16_t n_steps = constrain(String(parameters[0]).toInt(), 0, 65535);
  for(int i = 0; i < n_steps; i++){
    if(digitalRead(PIN_SWITCH_MAX) && motor_direction == LOW)  break;
    if(digitalRead(PIN_SWITCH_MIN) && motor_direction == HIGH) break;
    singleStep();
  }
}

void GetMotorDir(SCPI_C commands, SCPI_P parameters, Stream& interface) {
    interface.println(String(motor_direction, DEC));
}

void SetMotorDir(SCPI_C commands, SCPI_P parameters, Stream& interface) {
  if (parameters.Size() > 0) {
      uint8_t new_dir = constrain(String(parameters[0]).toInt(), 0, 1);
      set_direction(new_dir);
      if(_debug) interface.println(String(motor_direction, DEC));
  }
}

void GetMaxLimitState(SCPI_C commands, SCPI_P parameters, Stream& interface) {
  uint8_t state = digitalRead(PIN_SWITCH_MAX);
  if (state == HIGH) interface.println("HIGH");
  else               interface.println("LOW");
}

void GetMinLimitState(SCPI_C commands, SCPI_P parameters, Stream& interface) {
  uint8_t state = digitalRead(PIN_SWITCH_MIN);
  if (state == HIGH) interface.println("HIGH");
  else               interface.println("LOW");
}

/* Internal Operations */
void singleStep(){
  /*
   * Executes the pulse sequence for single microstep of the Monochromator stage. 
   * NOTE: An unsafe operation to run directly as it incorporates NO MONITORING of the stage limit switches.
   */

  /* Microstep pulse sequence */
  digitalWrite(PIN_STEP,HIGH);     
  delayMicroseconds(DEFAULT_STEP_DELAY);  
  digitalWrite(PIN_STEP,LOW);     
  delayMicroseconds(DEFAULT_STEP_DELAY);

  /* Keep track of the motor position */
  if(motor_direction) motor_position--;
  else                motor_position++;
}

void set_direction(uint8_t dir){
   motor_direction = dir;
   digitalWrite(PIN_DIR, motor_direction);
}
