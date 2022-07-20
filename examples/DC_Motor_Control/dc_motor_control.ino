#include <PIDControl.h>
#include "ESP32Servo.h"
#include "Encoder.h"

#define LED 13
#define POT 34
#define MOT 15

// rotary encoder
#define ENC_A 22
#define ENC_B 23

Servo brushed_dc;

Encoder encoder (ENC_A, ENC_B);

PIDControl pid;

float kp = 0.035;
float ki = 0.0002;
float kd = 0.15;


void calibrateEsc (void)
{
  brushed_dc.writeMicroseconds (2000);
  delay (2000);
  brushed_dc.writeMicroseconds (1000);
  delay (2000);

}
void setup() {

  // serial 
  Serial.begin (9600);

  // io
  pinMode (LED, OUTPUT);
  pinMode (POT, INPUT);

  //-------------------------------------------
  // servo
  //-------------------------------------------
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  brushed_dc.setPeriodHertz(50);      // Standard 50hz servo

  brushed_dc.attach (MOT, 1000, 2000);

  //// uncomment to calibrate esc if needed
  //calibrateEsc ();
  
  brushed_dc.writeMicroseconds (1500);
  delay (2000);
  //-------------------------------------------

  //-------------------------------------------
  // PID init sequence
  //-------------------------------------------
  pid.begin (kp, ki, kd);

  //-------------------------------------------
  
  for (int i = 0; i < 3; i++)
  {
    digitalWrite (LED, LOW);
    delay (30);
    digitalWrite (LED, HIGH);
    delay (30);
  }

}

void loop() 
{
  long pot_value;
  long esc;
  long enc_value;
  float pid_output;
  static long set_point = 0;
  static long last_pot_value = 0;
  
  enc_value = encoder.read ();

  pot_value = analogRead (POT);

  if (last_pot_value != pot_value)
  {
    // updates de set_point
    set_point = map (pot_value, 0, 4095, 0, 20000);
    last_pot_value = pot_value;
  }

  // maps pot from 0 to 20000 pulses (around 180 degrees)
  
  pid_output = pid.computePID (enc_value, set_point);

  float error = pid.getError ();

  esc = 1500 - pid_output;

/*  if (pid_output < 0)
    esc = 1500 - pid_output + 30;
  else
    esc = 1500 - pid_output - 30;
  */  

  if (esc > 2000) esc = 2000;
  if (esc < 1000) esc = 1000;
  
  Serial.print(set_point);
  Serial.print(",");
  Serial.print(enc_value);
  Serial.print(",");
  Serial.print(pid_output);
  Serial.println("");


  brushed_dc.writeMicroseconds (esc);

}