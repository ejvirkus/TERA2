#include <Stepper.h>
#include <IBusBM.h>

#include "Receiver.h"
#include "Steering_encoder.h"
#include "Drive_selector.h"
#include "Steering.h"

//Hall sensor pins:
#define L1 5
#define L2 6
#define L3 7

#define R1 2
#define R2 3
#define R3 4

ReceiverData receiver;
Steering_encoder_data encoder(30, 28, 32);
Drive_selector_switch drive_select;
Steering steering;
//Hall_sensor sensor;


int32_t wheel = 267;
int32_t fwd = 0;
bool ebrake = 1;
String received;

int FL_ticks = 0;
String prev_FL = "000";
int FR_ticks = 0;
String prev_FR = "000";

//init variable for serial comm
String data = "no info";
int values[2] = {328, 0};


void setup() {
  Serial.begin(115200);   //was 115200
  Serial.setTimeout(15);  //might need to be 40

  pinMode(L1, INPUT);
  pinMode(L2, INPUT);
  pinMode(L3, INPUT);
  pinMode(R1, INPUT);
  pinMode(R2, INPUT);
  pinMode(R3, INPUT);
  
  pinMode(10, INPUT);
}

void loop() {
  
  //serial communication
  if (Serial.available() > 0){

    data = Serial.readString();
    //Serial.println(data);
    
    values[0] = 328;
    values[1] = 0;
    int i = 0;
    char *ptr =strtok((char *)data.c_str(), ",");
    while (ptr != NULL && i < 2){
      values[i++] = atoi(ptr);
      ptr = strtok(NULL, ",");
    }
    //Serial.println(values[0]);
    
  }

  ebrake = digitalRead(10);
  
  //Defining RC remote info ranges
  int value1 = 268;
  //Read info from receiver-------------------------------
  value1 = receiver.readChannel(0, 365, 170, 0); //Oli 365, 170
  int value2 = receiver.readChannel(1, 255, -255, 0);
  int value3 = receiver.readChannel(2, 255, -255, 0);
  int value4 = receiver.readChannel(3, 255, -255, 0);
  int value5 = receiver.readSwitch(4, 1);
  int value6 = receiver.readChannel(5, 1,3, 2);
  int value8 = receiver.readSwitch(7, 1);
  /*
   * value1 = Steering 
   * value2 = Right joy up/down (Empty)
   * value3 = Throttle
   * value4 = Left joy left/right (Empty)
   * value5 = RC/TO switch
   * value6 = Transfer box (FWD/AWD/RWD)
   * value8 = Safety switch
   */

  //Reading encoder info
  int encoder_data = (encoder.readEncoder());

  //Drive function call
  //value 3 = RC throttle, value 6 = , values[1] = TO throttle, value5 = TO/RC switch, value8 = e turn off, ebrake = lidar info(brake)
  drive_select.Drive_mode(value3, value6, values[1], value5, value8, ebrake);

  //Steering function call
  steering.Left_Right(value1, value5, encoder_data , values[0]);

  //Read hall sensors call
  
  /*
  String FR = String(digitalRead(R1)) + String(digitalRead(R2)) + String(digitalRead(R3));
  
  if(prev_FR != FR){
    FR_ticks += 1;
    prev_FR = FR;
    //Serial.print(FR_ticks);
    //Serial.print(" ");
    //Serial.println(FR);
  }
  String FL = String(digitalRead(L1)) + String(digitalRead(L2)) + String(digitalRead(L3));
  
  if(prev_FL != FL){
    FL_ticks += 1;
    prev_FL = FL;
    //Serial.print(FL_ticks);
    //Serial.print(" ");
    //Serial.println(FL);
  }
  //Send odometry and steering angle to jetson:
  Serial.println(String(FR_ticks) + ", " + String(FL_ticks) + ", " + String(encoder_data));
  
  */
}
