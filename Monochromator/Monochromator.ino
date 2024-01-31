/*
 * For use in the Atomic Spectra experiment in McGill University physics course(s) PHYS-359/439.
 * Written by Brandon Ruffolo in 2023/2024.
 */

#include "Vrekrer_scpi_parser.h"

/* Serial COM parameters */
#define SKETCH_VERSION "0.0.9"
#define BAUD 115200 
#define LINEFEED "\n"

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
const uint16_t MICROSTEP_RATIO     = 400;   // Microstepping ratio (# microsteps/revolution) - Set on the motor driver
const uint16_t MAX_STEP            = 58870; // (VERIFIED EMPIRICALLY)
const uint16_t MIN_STEP            = 164;   // (VERIFIED EMPIRICALLY)
const uint16_t DEFAULT_STEP_DELAY  = 2000;  // Motor step pulse delay
const uint16_t HOMING_SETTLE_DELAY = 2000;  // Delay used between motor steps in the homing function
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
const char *HOMING_NAMES[] = {"Not Completed","Completed","Failed","Recalibration Required"};

bool _verbose = false;
bool _debug   = false;

SCPI_Parser Monochromator;

void setup(){
  /* SCPI commands */
  Monochromator.RegisterCommand(F("*IDN?")          , &Identify);
  //Monochromator.RegisterCommand(F("HELP")           , &Help);
  Monochromator.RegisterCommand(F("STATus?")        , &GetStatus);
  Monochromator.RegisterCommand(F("POSition?")      , &GetPosition);
  Monochromator.RegisterCommand(F("POSition")       , &SetPosition);
  Monochromator.RegisterCommand(F("HOME")           , &Home);
  Monochromator.RegisterCommand(F("DEBUG")          , &SetDebug);

  Monochromator.SetCommandTreeBase(F("STAGE"));
    Monochromator.RegisterCommand(F(":DIR?")        , &GetMotorDir);
    Monochromator.RegisterCommand(F(":DIR")         , &SetMotorDir);
    Monochromator.RegisterCommand(F(":STEP")        , &StepMotor);
    Monochromator.SetCommandTreeBase(F("STAGE:LIMit"));
      Monochromator.RegisterCommand(F(":MAX?")      , &GetMaxLimitState);
      Monochromator.RegisterCommand(F(":MIN?")      , &GetMinLimitState);
    Monochromator.SetCommandTreeBase(F("STAGE:MOVe"));
      Monochromator.RegisterCommand(F(":MAX")       , &GoToStageMax);
      Monochromator.RegisterCommand(F(":MIN")       , &GoToStageMin);

  Monochromator.SetCommandTreeBase(F("PMT"));
    Monochromator.RegisterCommand(F(":VALue?")      , &GetPMT);
    Monochromator.RegisterCommand(F(":OFFSET?")     , &GetPMTOffset);
    Monochromator.RegisterCommand(F(":OFFSET")      , &SetPMTOffset);


  /* Pin I/0 */
  pinMode(PIN_STEP,       OUTPUT); // Motor stepping pin
  pinMode(PIN_DIR,        OUTPUT); // Motor direction pin
  pinMode(PIN_OFFSET,     OUTPUT); // PMT I-to-V opamp offset bias voltage.
  pinMode(LED_BUILTIN,    OUTPUT); // LED for debugging
  pinMode(PIN_SWITCH_MAX, INPUT);  // Max limit switch monitor  
  pinMode(PIN_SWITCH_MIN, INPUT);  // Min limit switch monitor 
  
  analogWrite(PIN_OFFSET, pmt_offset);
  
  Serial.begin(BAUD);
}
void loop(){Monochromator.ProcessInput(Serial, LINEFEED);}


void Identify(SCPI_C commands, SCPI_P parameters, Stream& interface) {
  interface.println(F("Ugrad Labs, Monochromator Controller, v" SKETCH_VERSION ", " __DATE__));
  // "<vendor>,<model>,<serial number>,<firmware>"
}

/*
void Help(SCPI_C commands, SCPI_P parameters, Stream& interface) {
  interface.println(F("COMMAND LIST: \n"));
  interface.println(F("*IDN?        - Returns identifying information for the firmware on the device."));
  interface.println(F("STAT?        - Returns the homing status of the Monochromator stage."));
  interface.println(F("POS?         - Returns a floating point number representing the current position of the stage."));
  interface.println(F("POS x        - Sets the position of the stage. x is a user supplied floating point number representing the desired stage position."));
  interface.println(F("HOME         - Homes the Monochromator stage."));
  interface.println(F("PMT?         - Returns the current output of the PMT."));
  interface.println(F("PMT:OFFSET?  - Returns the current PMT offset setting."));
  interface.println(F("PMT:OFFSET x - Sets the PMT offset. x is a user supplied unsigned integer from 0 to 127."));
}*/

void GetStatus(SCPI_C commands, SCPI_P parameters, Stream& interface){
  interface.print(F("(Homing) "));
  interface.println(String(HOMING_NAMES[home_status]));
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
        home_status = FAILED;               // Signal that homing has failed
        interface.println("Homing Failed. Error code -1, see a technician for assistance."); 
        return;                       // Exit
      } 

      if( digitalRead(PIN_SWITCH_MAX) ) break; // Exit if switch was triggered (we're done stepping) 
      singleStep();                            // Step the motor
      delayMicroseconds(HOMING_SETTLE_DELAY);  // An additional delay to help microswitches settle
      
      if(Serial.available()){ /* Treat any incoming serial data as a signal to stop the move. */
        home_status = NOT_DONE;                // Signal that homing has not been completed
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
  if(home_status != FAILED){
    interface.println("Homing complete.");    // Give the ok over the serial line.
    home_status = COMPLETED;
  }
} 

void SetDebug(SCPI_C commands, SCPI_P parameters, Stream& interface){
  if (parameters.Size() > 0) {
    uint8_t setting = constrain(String(parameters[0]).toInt(), 0, 1);
    _debug = setting;
  } 
}

void GetPosition(SCPI_C commands, SCPI_P parameters, Stream& interface) {
  if(home_status == COMPLETED) interface.println(String(motor_position, DEC)); //interface.println(String((float)motor_position/MICROSTEP_RATIO, 4));
  else                         interface.println(F("Homing not complete, position unknown."));
}

void SetPosition(SCPI_C commands, SCPI_P parameters, Stream& interface) {
  uint16_t new_position;
  //float x;
  
  if (parameters.Size() > 0) {
    // x = constrain(String(parameters[0]).toFloat(), (float)MIN_STEP/MICROSTEP_RATIO, (float)MAX_STEP/MICROSTEP_RATIO );
    new_position = constrain(String(parameters[0]).toInt(), 0, 65535);
    if(_debug) interface.println("Target: "+String(new_position, DEC));
  }
  else{
    interface.println(F("No parameter supplied."));
    return;
  }
  
  if(home_status != COMPLETED){
    interface.println(F("Homing not completed. Position cannot be set."));
    return;
  }
  if(new_position > MAX_STEP || new_position < MIN_STEP ){
    interface.println(F("Position is out of range."));
    return;  
  }
  if(new_position > motor_position)      set_direction(LOW);  // Set to forward direction 
  else if(new_position < motor_position) set_direction(HIGH); // Set to reverse direction
  else{                                                       // Exit (we are already there)
    interface.println(F("Already at desired position."));
    return;
  }                                                 

  if(_verbose) interface.println(F("Move Starting.")); // Indicate start of stage movement

  /* Step until we arrive at desired position */
  while(motor_position != new_position){ 
    
    /* Check limit switches (in case something is horribly wrong with the calibration!) */
    if (digitalRead(PIN_SWITCH_MAX) && motor_direction == LOW){  // MAX limit switch hit before expected.
      home_status = RECAL;                                       // Call for a recalibration of the stage
      interface.println(F("MAX limit switch hit before expected. Stage recalibration needed."));
      return; }
    if (digitalRead(PIN_SWITCH_MIN)&& motor_direction == HIGH){  // MIN limit switch hit before expected.
      home_status = RECAL;                                       // Call for a recalibration of the stage
      interface.println(F("MIN limit switch hit before expected. Stage recalibration needed."));
      return;}

    
    if(Serial.available()){    /* Treat any incoming serial data as a signal to stop the move. */
      interface.println(F("Move interrupted."));
      return;
    }
    singleStep();
  }
  interface.println(F("Move Completed."));
}

void GetPMT(SCPI_C commands, SCPI_P parameters, Stream& interface) {
  uint8_t pmt_val = analogRead(PIN_PMT);
  interface.println(String(pmt_val, DEC));
}

void SetPMTOffset(SCPI_C commands, SCPI_P parameters, Stream& interface) {
   if (parameters.Size() > 0) {
    pmt_offset = constrain(String(parameters[0]).toInt(), 0, 127);
    analogWrite(PIN_OFFSET, pmt_offset);
    interface.println(String(pmt_offset, DEC));
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
  if(motor_direction == LOW) interface.println("LOW");
  else                       interface.println("HIGH");
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

void GoToStageMax(SCPI_C commands, SCPI_P parameters, Stream& interface) {
  set_direction(LOW);                               // Set the movement correct direction 
  if(_verbose) interface.println(F("Move Starting.")); // Indicate start of stage movement

  while(1){ 
    if( digitalRead(PIN_SWITCH_MAX) ) break; // Exit if switch was triggered (we're done stepping) 
    singleStep();                            // Step the motor
    delayMicroseconds(HOMING_SETTLE_DELAY);  // An additional delay to help microswitches settle
    
    if(Serial.available()){ /* Treat any incoming serial data as a signal to stop the move. */
      interface.println(F("Move interrupted.")); // Indicate interruption of stage movement
      return;                    
    }
  }
  interface.println(F("Move Completed.")); // Indicate successful end of stage movement
}

void GoToStageMin(SCPI_C commands, SCPI_P parameters, Stream& interface) {
  set_direction(HIGH);                              // Set the movement correct direction 
  if(_verbose) interface.println(F("Move Starting.")); // Indicate start of stage movement

  while(1){ 
    if( digitalRead(PIN_SWITCH_MIN) ) break; // Exit if switch was triggered (we're done stepping) 
    singleStep();                            // Step the motor
    delayMicroseconds(HOMING_SETTLE_DELAY);  // An additional delay to help microswitches settle
    
    if(Serial.available()){ /* Treat any incoming serial data as a signal to stop the move. */
      interface.println(F("Move interrupted.")); // Indicate interruption of stage movement
      return;                    
    }
  }
  interface.println(F("Move Completed.")); // Indicate successful end of stage movement
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
