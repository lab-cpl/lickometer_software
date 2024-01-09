/*
Author: WAC@IOWLABS
VERSION: V0.0.2

NAME:
DESCRPIPTIONS:
Se arregla error en que los licks del time out contaban para gatillar un evento

2023-05-23
- Every time a choice is made -> lights off + timeout
- Use a third sensor as a nosepoke

2024-01-09
- changes so any protocol could be used in any of the spouts

LIBS:

TODO:

*/
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPR121.h>
#include <Adafruit_MotorShield.h>
#include <ArduinoJson.h>

// Version (mayor, minor, patch, build)
#define VERSION    "licometro - v.0.2.6.b.1"

#define CALIBRATION 0
#define RUN         1

#define ID          "a01" // CAMBIAR SI HAY DISTINTOS ARDUINOS
#define DEBUG       0      // habilita los mensajes de consola para debuggear
#define RESET       asm("jmp 0x0000")
#define TYPE_DATA   "data"
#define TYPE_RESP   "response"
#define N_SENSORS   2 // spout 0, spout 2, nosepoke is not on mpr
#define N_LEDS      3
#define N_BLINKS    3
#define BLOCK_TIME  5000
#define NOSE_POKE_PIN 7
#define NOSE_POKE   2  // nosepoke is a special sensor
#define NOSE_POKE_TIME_IN   50  //  50 ms to triggers an election trial

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
const size_t capacity_tx = JSON_OBJECT_SIZE(11);
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

unsigned long licks_counter[]   = {0,0}; // store the total number of licks made
unsigned long licks_counter_valid[]   = {0,0}; // store the number of licks that are valid
unsigned long events_counter[]  = {0,0}; // store the number of times that "must" be triggered an event
unsigned long success_counter[] = {0,0}; // store the numbers of events that really happen
bool trial_end[]                =  {0, 0};
bool licks_triggered[]          = {0,0}; // store if the sensor was triggered; 0 not triggered, 1 triggered;
// 0, 1 = spouts
// 2 = nosepoke
bool bussy_sensors[]            = {0,0,0}; // indicate if the sensors is blocked; 0 no blocked, 1 blocked;
// 0, 1 = spouts
// 2 = nosepoke
long start_times[]              = {0,0,0}; // store de duratrion of the blocked time;
// 0, 1 = spouts
// 2 = nosepoke
bool sensors_state[]            = {0,0}; // indicates if the sensor is being pressed or not
bool nosePoke_state             = false;
unsigned long nose_in_timestamp = 0;
bool nosePoke_touched                = 1;
long unsigned int nose_in_var   = 0;
bool nose_in_var_bool           = false;

//CAMBIAR ESTA VARIABLE CON LOS SENSORES USADOS {SENSOR_0,SENSOR_1}
// nosepoke is not here because is not part of the mpr
uint8_t active_sensor_index[]   = {0,2}; // save the index of the actual ussed sensors
uint8_t licks_threshold[]        = {5,5}; //
int licks_pr[180]              = {5, 7, 11, 17, 25, 35, 47, 61,
                                  77, 95, 115, 137, 161, 187, 215, 245,
                                  277, 311, 347, 385, 425, 467, 511, 557,
                                  605, 655, 707, 761, 817, 875, 935, 997,
                                  1061, 1127, 1195, 1265, 1337, 1411, 1487, 1565,
                                  1645, 1727, 1811, 1897, 1985, 2075, 2167, 2261,
                                  2357, 2455, 2555, 2657, 2761, 2867, 2975, 3085,
                                  3197, 3311, 3427, 3545, 3665, 3787, 3911, 4037,
                                  4165, 4295, 4427, 4561, 4697, 4835, 4975, 5117,
                                  5261, 5407, 5555, 5705, 5857, 6011, 6167, 6325,
                                  6485, 6647, 6811, 6977, 7145, 7315, 7487, 7661,
                                  7837, 8015, 8195, 8377, 8561, 8747, 8935, 9125,
                                  9317, 9511, 9707, 9905, 10105, 10307, 10511, 10717,
                                  10925, 11135, 11347, 11561, 11777, 11995, 12215, 12437,
                                  12661, 12887, 13115, 13345, 13577, 13811, 14047, 14285,
                                  16517, 16775, 17035, 17297, 17561, 17827, 18095, 18365,
                                  18637, 18911, 19187, 19465, 19745, 20027, 20311, 20597,
                                  20885, 21175, 21467, 21761, 22057, 22355, 22655, 22957,
                                  23261, 23567, 23875, 24185, 24497, 24811, 25127, 25445,
                                  25765, 26087, 26411, 26737, 27065, 27395, 27727, 28061,
                                  28397, 28735, 29075, 29417, 29761, 30107, 30455, 30805,
                                  31157, 31511, 31867, 32225
                                 };
// increasing entropy first 100,100 is the 15 min time gap
// that is why they are repeated
//uint8_t events_probability[]    = {100,100, 100,100, 100,50, 25,75, 50,50};

// constant minimal entropy
uint8_t events_probability[]    = {100,100, 100,100, 100,100, 100,100, 100,100};
// decreaseing entropy
//uint8_t events_probability[]    = {100,100, 50,50, 25,75, 100,50, 100,100};

//CAMBIAR ESTA VARIABLE CON LOS LEDS USADOS {LED_0,LED_1}
uint8_t leds_pins[]             = {3,6,10};
uint8_t leds_status[]           = {0,0,0};

int probability     = 100;
uint8_t temp_index  = 0;

int led_power     = 10;
double powerMotor = 0.6;
// important variable 12 is default
int motor_steps   = 24;

// VARIABLES RELACIONADAS A CAMBIAR LA PROBABILIDAD DE LOS SPOUTS
#define BIN_TIME 900000
int n_bin = 0;
int n_bin_last    = 0;
long time_session  = 0; // only starts after first triggered event, either spout is ok
long time_current = 0;
long time_start   = 0;
long time_last    = 0;

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
  pinMode(13,OUTPUT);
  pinMode(NOSE_POKE_PIN, INPUT);
  Serial.begin(115200);

  //Inicialize the LEDs
  for(int i = 0; i < N_LEDS ;i++)
  {
    pinMode(leds_pins[i],OUTPUT);
  }

  //Inicialize the capacitive sensor
  if(DEBUG){Serial.println("Adafruit MPR121 Capacitive Touch sensor test");}
  if (!capacitive_sensor.begin(0x5A))
  {
    Serial.println("MPR121 not found, check wiring?");
    while (1);
  }
  if(DEBUG){Serial.println("MPR121 found!");}

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
  if(state == CALIBRATION)
  {
    calibration();
  }
  if(state == RUN)
  {
    // detectar en que BIN estamos
    // the first detected bin is not relevant
    // calculations are relevant after the first triggered event
    n_bin = (int)((millis()-time_session)/BIN_TIME); //entrega el bin (0,inf)
    if (n_bin>4){
      n_bin= 4;
      }
    readSensor();
    nose_in_var = 0;
    nose_in_var_bool = false;
    // if an event is triggered turn of spout leds
    // and turn on nosepoke led for next trial
    if (trial_end[0] || trial_end[1]){
      bussy_sensors[0] = 1;
      bussy_sensors[1] = 1;
      //turnAllLeds(0);
      analogWrite(leds_pins[0], 0);
      analogWrite(leds_pins[1], 0);
      analogWrite(leds_pins[2], led_power);
    }
    else{
      bussy_sensors[0] = 0;
      bussy_sensors[1] = 0;
      //turnAllLeds(1);
      analogWrite(leds_pins[0], led_power);
      analogWrite(leds_pins[1], led_power);
      analogWrite(leds_pins[2], 0);
    }
    for(int i = 0; i<N_SENSORS;i++)
    {
      if(licks_triggered[i])
      {
        if( licks_counter_valid[i] % licks_threshold[i] == 0)
        {
          if(!bussy_sensors[i])
          {
            events_counter[i] += 1;
            bussy_sensors[i]   = 1;
	    // if this is a PR protocol then licks_threshold should go up 
	    // for the selected spout
	    licks_threshold[i] = licks_pr[events_counter[i]];
	    licks_counter_valid[i] = 0;
	    // set time when event is triggered
	    // set this so both sensor go to block time
	    // 0 and 1 should be both spouts
            start_times[i]     = millis();
	    if (time_session == 0){
		    time_session = millis();
	    }
	    // probability move in pairs
	    // n_bin = 0, index 0 and 1
	    // n_bin = 1, index 2 and 3
	    // n_bin = 2, index 4 and 5
	    // n_bin = 3, index 6 and 7
	    // n_bin = 4, index 8 and 9
            probability = random(int(100/events_probability[((n_bin)*2)+i]));
            if(!probability)
            {
              success_counter[i]+=1;
              // trial ends here
              trial_end[i] = 1;
              //digitalWrite(leds_pins[0], 0);
              //digitalWrite(leds_pins[1], 0);
	      // turns both leds off
	      //analogWrite(leds_pins[i],0);
	      // delivers reward
	      // note this is code blocking
              Motor_array[i]->step(motor_steps, FORWARD, MICROSTEP); // motor ON
              Motor_array[i]->release();
            }
          }
        }
        publishSensor(i);
        licks_triggered[i] = 0;
      }
      // because both spouts go to block time at the same time
      // if any of start times is greater than block time
      // then both spouts should be available for a new event
      readNosePoke();
      //Serial.println(nose_in_var);
      if(!nosePoke_state)
      {
	      // sets both spout available to a new event
        //bussy_sensors[i] = 0;
        // if choosen spout ends block time the trial re-starts
        start_times[NOSE_POKE] = millis();
        nose_in_var = start_times[NOSE_POKE] - nose_in_timestamp;
        nose_in_var_bool = true;
        //Serial.println(nose_in_var);
        if(nose_in_var >= NOSE_POKE_TIME_IN){
		// starts the new trial
          trial_end[0] = 0;
          trial_end[1] = 0;
        }
	// publish 9 as nosepoke
	publishSensor(9);
	    // turns both lights on
	     // analogWrite(leds_pins[i],led_power);
      }
    }
  }
}

void calibration()
{
  current_touched = capacitive_sensor.touched();
  for (uint8_t i=0; i < N_SENSORS; i++)
  {
    temp_index = active_sensor_index[i];
    if((current_touched & _BV(temp_index)) && !(last_touched & _BV(temp_index)) )
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

void readNosePoke(){
    nosePoke_state = digitalRead(NOSE_POKE_PIN);
    if(!nosePoke_state && nosePoke_state != nosePoke_touched){
      nose_in_timestamp = millis();
      }
    nosePoke_touched = nosePoke_state;
}

void readSensor()
{
  current_touched = capacitive_sensor.touched();
  for (uint8_t i=0; i < N_SENSORS; i++)
  {
    temp_index = active_sensor_index[i];
    if((current_touched & _BV(temp_index)) && !(last_touched & _BV(temp_index)) )
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

      // If sensor is not busy, then licks do not count to trigger an event, otherwise, do not register
      if (!bussy_sensors[i]) {
        licks_counter_valid[i] += 1;
        }
      }
      //publishSensor(i);
  }
  last_touched = current_touched;
}

void blinkTubeLights(int Delay, boolean flag)
{

  for (int i=0; i < N_BLINKS; i++)
  {
    for (int q = 0; q < N_LEDS; q++) { analogWrite(leds_pins[q],0); }
    delay(Delay);
    
    if (flag)
    {
      for (int q = 0; q < N_LEDS; q++){analogWrite(leds_pins[q],25);}
    }
    else
    {
      for (int q = 0; q < N_LEDS; q++)
      {
        if (licks_triggered[q]) { analogWrite(leds_pins[q],25); }
      }
    }
    delay(Delay);
    
  }
}

void turnAllLeds(bool onoff)
{
  if(onoff){ for(int i = 0; i < N_LEDS ; i++){analogWrite(leds_pins[i],led_power);}}
  else{      for(int i = 0; i < N_LEDS ; i++){digitalWrite(leds_pins[i],0);}}
}

void publishSensor(int index)
{
	// if reading comes from nosepoke
	if(index == 9){
	  doc_tx["id"]      = ID;
	  doc_tx["type"]    = state;
	  doc_tx["sensor"]  = 2;
	  doc_tx["time"]    = millis();
	  doc_tx["lick"]    = -1;
	  doc_tx["event"]   = trial_end[0] == trial_end[1]; // 1 = new valid trial
	  doc_tx["success"] = nose_in_var; // if event == 1, this is the required time inside nosepoke to trigger next trial
	  doc_tx["activity"]  = 0;
	}
	else{
	  doc_tx["id"]      = ID;
	  doc_tx["type"]    = state;
	  doc_tx["sensor"]  = index;
	  doc_tx["time"]    = millis();
	  doc_tx["lick"]    = licks_counter[index];
	  doc_tx["event"]   = events_counter[index];
	  doc_tx["success"] = success_counter[index];
	  doc_tx["activity"]  = sensors_state[index];
	}
  //check if python interface script reads this
  //doc_tx["nosepoke"] = nose_in_var_bool;
  //doc_tx["probability_pair"] = events_probability[((n_bin)*2)+index];
  //doc_tx["n_bin"] = n_bin;

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
    if(strcmp(cmd,"mode")==0)
    {
      if( arg == 1 )
      {

        for (size_t i = 0; i < N_SENSORS; i++)
        {
            licks_counter[i] = 0;
        }
        state =  RUN;
        if(DEBUG){Serial.println("mode runnning");}
      }
      else
      {
        state = CALIBRATION;
        if(DEBUG){Serial.println("mode calibration");}
      }
    }
    else if(strcmp(cmd,"ledOn")==0)
    {
      if(arg<N_LEDS){digitalWrite(leds_pins[arg],HIGH);}
    }
    else if(strcmp(cmd,"ledOff")==0)
    {
      if(arg<N_LEDS){digitalWrite(leds_pins[arg],LOW);}
    }
    else if(strcmp(cmd,"test")==0)
    {
      if(arg==1)
      {
          blinkTubeLights(DELAY_TEST_LEDS, 1);

      }
      if(arg==2)
      {
        digitalWrite(leds_pins[0],LOW);
        Motor_array[0]->step(motor_steps, FORWARD, MICROSTEP); // motor ON
        Motor_array[0]->release();
        digitalWrite(leds_pins[0],HIGH);
      }
      if(arg==3)
      {
        digitalWrite(leds_pins[1],LOW);
        Motor_array[1]->step(motor_steps, FORWARD, MICROSTEP); // motor ON
        Motor_array[1]->release();
        digitalWrite(leds_pins[1],HIGH);
      }
      if(arg==4)
      {
        digitalWrite(leds_pins[0],LOW);
        Motor_array[0]->step(motor_steps, FORWARD, MICROSTEP); // motor ON
        Motor_array[0]->release();
        digitalWrite(leds_pins[0],HIGH);
        delay(DELAY_TEST_MOTORS);
        digitalWrite(leds_pins[1],LOW);
        Motor_array[1]->step(motor_steps, FORWARD, MICROSTEP); // motor ON
        Motor_array[1]->release();
        digitalWrite(leds_pins[1],HIGH);
        delay(DELAY_TEST_MOTORS);
      }
    }
    else if(strcmp(cmd,"reset_data")==0)
    {
      for (int i = 0; i < N_SENSORS ; i++)
      {
        licks_counter[i]    = 0;
        events_counter[i]   = 0;
        success_counter[i]  = 0;
      }
    }
    else if(strcmp(cmd,"reset")==0)
    {
      RESET;
    }
    else
    {
      if(DEBUG){Serial.println("Command not valid");}
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
