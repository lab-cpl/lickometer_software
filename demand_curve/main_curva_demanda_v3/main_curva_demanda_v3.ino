  /*
  Author: WAC@IOWLABS
  VERSION: V0.0.1

  NAME: main curva demanda v1
  DESCRPIPTIONS:

  LIBS:

  TODO:

*/
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPR121.h>
#include <Adafruit_MotorShield.h>
#include <ArduinoJson.h>
#include <ezButton.h>

// Version (mayor, minor, patch, build)
#define VERSION    "licometro - v.0.2.6.b.1"

#define CALIBRATION 0
#define RUN         1

#define ID          "a04" // CAMBIAR SI HAY DISTINTOS ARDUINOS
#define DEBUG       0      // habilita los mensajes de consola para debuggear
#define RESET       asm("jmp 0x0000")
#define TYPE_DATA   "data"
#define TYPE_RESP   "response"
#define N_SENSORS   2
#define N_LEDS      2
#define N_BLINKS    3
#define BLOCK_TIME  20000
#define BIN_TIME    600000 //tamaño de los bins de tiempo (1 min= 60000ms)

// Variable related to pause function
#define BUTTON_PIN      11
ezButton toggleSwitch(BUTTON_PIN); // set the pin to use
bool PAUSE                = false;
unsigned long P_START	  = 0;
unsigned long P_TOTAL     = 0;
unsigned long P_CURRENT   = 0;

#define DELAY_TEST_LEDS   2000 //2 seg
#define DELAY_TEST_MOTORS 3000 //3 seg

#ifndef _BV
#define _BV(bit) (1 << (bit)) //macro para enmascarar bits
#endif

/*----------------
    INSTANCES
  ------------------*/
Adafruit_MPR121      capacitive_sensor = Adafruit_MPR121();      // capacitive sensor instance
Adafruit_MotorShield motor_shield      = Adafruit_MotorShield(); // Motor instance

Adafruit_StepperMotor*  Motor_array[2]; // Declare array of motor
Adafruit_StepperMotor   *Motor1 = motor_shield.getStepper(200, 1); // ports M1 and M2 of motorshield
Adafruit_StepperMotor   *Motor2 = motor_shield.getStepper(200, 2); // ports M3 and M4 of motorshield


//JSON RECIVE
const size_t capacity_rx = JSON_OBJECT_SIZE(2) + 20;
DynamicJsonDocument doc_rx(capacity_rx);
const char* json_rx = "{\"cmd\":\"gps\",\"arg\":1}";
DeserializationError error_rx;

//JSON SEND
const size_t capacity_tx = JSON_OBJECT_SIZE(8);
DynamicJsonDocument doc_tx(capacity_tx);
//PARSE VAR
const char* cmd;
int arg = 0;

/*----------------
    VARIABLES
  ------------------*/
bool state  = RUN;

uint16_t last_touched    = 0;
uint16_t current_touched = 0;

unsigned long licks_counter[]   = {0, 0}; // store the number of licks
unsigned long licks_events_pr[] = {0, 0}; // licks para la cuenta del pr
unsigned long events_counter[]  = {0, 0}; // store the number of times that "must" be triggered an event
unsigned long success_counter[] = {0, 0}; // store the numbers of events that really happen
bool licks_triggered[]          = {0, 0}; // store if the sensor was triggered; 0 not triggered, 1 triggered;
bool bussy_sensors[]            = {0, 0}; // indicate if the sensors is blocked; 0 no blocked, 1 blocked;
long start_times[]              = {0, 0}; // store de duratrion of the blocked time;
bool sensors_state[]            = {0, 0}; // indicates if the sensor is being pressed or not

//CAMBIAR ESTA VARIABLE CON LOS SENSORES USADOS {SENSOR_0,SENSOR_1}
uint8_t active_sensor_index[]   = {0, 2}; // save the index of the actual ussed sensors


//Yani cambiar este dato por sensor 0 o 1 segun donde este la sacarosa. 
uint8_t pr_sensor = 1; // sensor number with progressive ratio 0 = 0; 1 = 2 (sensor marcado como 2 en el licometro)


int licks_treshold[]        = {5, 5}; //
int licks_pr[6]              = {5, 10, 20, 40, 80, 120}; //number of licks required each bin. Last value is repeated so comparisons can be made.
uint8_t events_probability[]    = {100, 100};

// ESTA VARIABLE SE PONE PARA INDICAR EN QUE SPOUT ESTA LA SACAROSA
// 0 sucrose_spout indica el puerto 0 del licometro
// 1 sucrose_spout indica el puerte 2 del licometro
int sucrose_spout = 0;

//CAMBIAR ESTA VARIABLE CON LOS LEDS USADOS {LED_0,LED_1}
uint8_t leds_pins[]             = {3, 6};
uint8_t leds_status[]           = {0, 0};

int probability     = 100;
uint8_t temp_index  = 0;
int n_bin  = 0;
int n_bin_last  = 0;

long time_session   = 0;
long time_current   = 0;
long time_start     = 0;
long time_last      = 0;

int led_power     = 10;
double ledPowerMod[] = {1.0, 0.1};
int brightness;
double powerMotor = 0.6;
int motor_steps   = 12;

/*----------------
    FUNCTIONS
  ------------------*/

void processCmd();
void publishSensor(int index);
void calibration();
void readSensor();
void blinkTubeLights(int Delay, boolean flag);
void turnAllLeds(bool onoff);

void setup()
{
  pinMode(13, OUTPUT);
  toggleSwitch.setDebounceTime(100); // debounce for on/off switch
  Serial.begin(115200);

  //Inicialize the LEDs
  for (int i = 0; i < N_LEDS ; i++)
  {
    pinMode(leds_pins[i], OUTPUT);
  }

  //Inicialize the capacitive sensor
  if (DEBUG) {
    Serial.println("Adafruit MPR121 Capacitive Touch sensor test");
  }
  if (!capacitive_sensor.begin(0x5A))
  {
    Serial.println("MPR121 not found, check wiring?");
    while (1);
  }
  if (DEBUG) {
    Serial.println("MPR121 found!");
  }

  capacitive_sensor.writeRegister(MPR121_ECR, 0x00);  // Adjust threshold for MPR121
  capacitive_sensor.setThresholds(0x14, 0x07);        // This is the threshold that was found not to cause any false positives
  capacitive_sensor.writeRegister(MPR121_ECR, 0x8F);

  //Initialize the motor shield
  motor_shield.begin();
  Motor_array[0] = Motor1;
  Motor_array[1] = Motor2;
  Motor_array[0] -> setSpeed(50); // 50 revevolutions per second
  Motor_array[1] -> setSpeed(50); // 50 revevolutions per second

  turnAllLeds(1);
  //digitalWrite(leds_pins[0],1);
  //digitalWrite(leds_pins[1],1);
}

void loop()
{
  if (state == CALIBRATION)
  {
    calibration();
  }
  if (state == RUN)
  {
    // Following chunks deal with PAUSE

    // check toggle switch
    toggleSwitch.loop();
    int switchState = toggleSwitch.getState();
    if (switchState == HIGH){
      PAUSE = false;
    }
    else {
      PAUSE = true;
    }

    if (PAUSE == true){
      //Serial.println(millis() - (P_TOTAL + P_CURRENT) - time_session );
      pauseLights();
      if (P_START == 0){ // evaluates only once
	      P_START = millis();
      }
      P_CURRENT = millis() - P_START; // compute the length of the pause
    }
    if (PAUSE == false){
      if (P_START != 0){ // when coming from a previuos pause
      	P_TOTAL = P_TOTAL + P_CURRENT; // total paused length
      	publishPause(); // before reseting the p_current send the length of the previous pause
      	P_START = 0; // reset p_start
      	P_CURRENT = 0; // reset p_current
      }
    }
    // detectar en que BIN estamos
    // note that when is UNPAUSED P_CURRENT = 0
    // when is paused P_CURRENT updates constantly
    n_bin = (int)(( (millis() - (P_TOTAL + P_CURRENT)) - time_session)/BIN_TIME); //entrega el bin (0,inf)
    if (n_bin>5){
      n_bin= 5;
      }
      // only read sensor when there is no pause
      if (PAUSE == false){
        readSensor();
      }
    for (int i = 0; i < N_SENSORS; i++)
    {
      if (i == pr_sensor) {
        licks_treshold[i] = licks_pr[n_bin]; // el nuevo treshold será el que corresponde en la lista               
              }
      if (licks_triggered[i])
      {
        if ( licks_events_pr[i] % licks_treshold[i] == 0)
        {
          if (!bussy_sensors[i])
          {
            events_counter[i] += 1;
            bussy_sensors[i]   = 1;
            start_times[i]     = millis();
            
            if (time_session == 0){
                time_session = millis(); // si time_session == 0, entonces significa que nunca se ha modificado, por lo que pone el primer millis desde que se corre el lickómetro.
            }

            probability = random(int(100 / events_probability[i]));
            if (!probability)
            {
              success_counter[i] += 1;
              analogWrite(leds_pins[i], 0);
              Motor_array[i]->step(motor_steps, FORWARD, MICROSTEP); // motor ON
              Motor_array[i]->release();
            }
          }
        }
        if (PAUSE == false){
          publishSensor(i);
        }
        licks_triggered[i] = 0;
      }
      if ( millis() - start_times[i] >  BLOCK_TIME )
      {
        bussy_sensors[i] = 0;
	if (i == sucrose_spout)
	{
		brightness = led_power * ledPowerMod[0];
	}
	else{
		brightness = led_power * ledPowerMod[1];
	}
        analogWrite(leds_pins[i], brightness);
      }

    }
}
}

void calibration()
{
  current_touched = capacitive_sensor.touched();
  for (uint8_t i = 0; i < N_SENSORS; i++)
  {
    temp_index = active_sensor_index[i];
    if ((current_touched & _BV(temp_index)) && !(last_touched & _BV(temp_index)) )
    {
      sensors_state[i] = 1;
      publishSensor(i);
    }
    if (!(current_touched & _BV(temp_index)) && (last_touched & _BV(temp_index)) )
    {
      sensors_state[i]  = 0;
      publishSensor(i);
    }
  }
  last_touched = current_touched;
}

void readSensor()
{
  // detectar si hubo cambio de BIN
  if(n_bin != n_bin_last){
      licks_events_pr[0]=0;
      licks_events_pr[1]=0;
      start_times[0]=0;
      start_times[1]=0;
      bussy_sensors[0] = 0;
      bussy_sensors[1] = 0;
      n_bin_last=n_bin;
    }
  current_touched = capacitive_sensor.touched();
  for (uint8_t i = 0; i < N_SENSORS; i++)
  {
    temp_index = active_sensor_index[i];
    if ((current_touched & _BV(temp_index)) && !(last_touched & _BV(temp_index)) )
    {
      sensors_state[i] = 1;
      publishSensor(i);
    }
    // Update sensor off
    if (!(current_touched & _BV(temp_index)) && (last_touched & _BV(temp_index)) )
    {
      licks_triggered[i] =  1;
      sensors_state[i]   =  0;
      licks_counter[i]   += 1;
      if(!bussy_sensors[i]){
        licks_events_pr[i]++;
      }
      //publishSensor(i);
    }
  }
  last_touched = current_touched;
}

void pauseLights()
{
  for (int i = 0; i < N_LEDS; i++)
  {
    analogWrite(leds_pins[i], 0);
    delay(25);
    analogWrite(leds_pins[i], 25);
    delay(25);
  }
}

void blinkTubeLights(int Delay, boolean flag)
{

  for (int i = 0; i < N_BLINKS; i++)
  {
    for (int q = 0; q < N_LEDS; q++) {
      analogWrite(leds_pins[q], 0);
    }
    delay(Delay);

    if (flag)
    {
      for (int q = 0; q < N_LEDS; q++) {
        analogWrite(leds_pins[q], 25);
      }
    }
    else
    {
      for (int q = 0; q < N_LEDS; q++)
      {
        if (licks_triggered[q]) {
          analogWrite(leds_pins[q], 25);
        }
      }
    }
    delay(Delay);

  }
}

void turnAllLeds(bool onoff)
{
  for (uint8_t i=0; i < N_SENSORS; i++){
	if (i == sucrose_spout)
	{
		brightness = led_power * ledPowerMod[0];
	}
	else{
		brightness = led_power * ledPowerMod[1];
	}
        analogWrite(leds_pins[i], brightness);
  }
}

void publishSensor(int index)
{
  doc_tx["id"]      = ID;
  doc_tx["type"]    = state;
  doc_tx["sensor"]  = index;
  doc_tx["time"]    = millis();
  doc_tx["lick"]    = licks_counter[index];
  doc_tx["event"]   = events_counter[index];
  doc_tx["success"] = n_bin; // gets current bin instead of success, as success is always 100%
  doc_tx["activity"]  = sensors_state[index];

  serializeJson(doc_tx, Serial);
  Serial.println("");
}

// this function sends a row to identify that a pause was performed
// -1 in all columns means there was a pause
void publishPause()
{
  doc_tx["id"]      = "PAUSE";
  doc_tx["type"]    = state;
  doc_tx["sensor"]  = -1;
  doc_tx["time"]    = P_CURRENT; // Length of the pause in milliseconds
  doc_tx["lick"]    = -1;
  doc_tx["event"]   = -1;
  doc_tx["success"] = -1;
  doc_tx["activity"]  = -1;

  serializeJson(doc_tx, Serial);
  Serial.println("");
}

void processCmd()
{
  //check for error
  error_rx = deserializeJson(doc_rx, Serial);
  if (error_rx)
  {
    Serial.println("testing");
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error_rx.c_str());
    return;
  }

  //parsing incoming msg
  cmd = doc_rx["cmd"];
  arg = doc_rx["arg"];

  //if(DEBUG){Serial.println(cmd);}
  //if(DEBUG){Serial.println(arg);}

  //prossesing incoming command
  if (strcmp(cmd, "mode") == 0)
  {
    if ( arg == 1 )
    {

      for (size_t i = 0; i < N_SENSORS; i++)
      {
        licks_counter[i] = 0;
      }
      state =  RUN;
      if (DEBUG) {
        Serial.println("mode runnning");
      }
    }
    else
    {
      state = CALIBRATION;
      if (DEBUG) {
        Serial.println("mode calibration");
      }
    }
  }
  else if (strcmp(cmd, "ledOn") == 0)
  {
    if (arg < N_LEDS) {
      digitalWrite(leds_pins[arg], HIGH);
    }
  }
  else if (strcmp(cmd, "ledOff") == 0)
  {
    if (arg < N_LEDS) {
      digitalWrite(leds_pins[arg], LOW);
    }
  }
  else if (strcmp(cmd, "test") == 0)
  {
    if (arg == 1)
    {
      blinkTubeLights(DELAY_TEST_LEDS, 1);

    }
    if (arg == 2)
    {
      digitalWrite(leds_pins[0], LOW);
      Motor_array[0]->step(motor_steps, FORWARD, MICROSTEP); // motor ON
      Motor_array[0]->release();
      digitalWrite(leds_pins[0], HIGH);
    }
    if (arg == 3)
    {
      digitalWrite(leds_pins[1], LOW);
      Motor_array[1]->step(motor_steps, FORWARD, MICROSTEP); // motor ON
      Motor_array[1]->release();
      digitalWrite(leds_pins[1], HIGH);
    }
    if (arg == 4)
    {
      digitalWrite(leds_pins[0], LOW);
      Motor_array[0]->step(motor_steps, FORWARD, MICROSTEP); // motor ON
      Motor_array[0]->release();
      digitalWrite(leds_pins[0], HIGH);
      delay(DELAY_TEST_MOTORS);
      digitalWrite(leds_pins[1], LOW);
      Motor_array[1]->step(motor_steps, FORWARD, MICROSTEP); // motor ON
      Motor_array[1]->release();
      digitalWrite(leds_pins[1], HIGH);
      delay(DELAY_TEST_MOTORS);
    }
  }
  else if (strcmp(cmd, "reset_data") == 0)
  {
    for (int i = 0; i < N_SENSORS ; i++)
    {
      licks_counter[i]    = 0;
      events_counter[i]   = 0;
      success_counter[i]  = 0;
    }
  }
  else if (strcmp(cmd, "reset") == 0)
  {
    RESET;
  }
  else
  {
    if (DEBUG) {
      Serial.println("Command not valid");
    }
  }

  cmd = "";
}

void serialEvent()
{

  if (Serial.available() > 0)
  {
    processCmd();
  }
}
