#include "Drive_selector.h"
#include "Arduino.h"

Drive_selector_switch::Drive_selector_switch()
{
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
}

//value 3 = RC throttle, value 6 = , throttle = TO throttle, value5 = TO/RC switch, value8 = e turn off, ebrake = lidar info(brake)
int Drive_selector_switch::Drive_mode(int value3, int value6, int throttle, int value5, int value8, int ebrake)
{
  //TO mode
      if(value5 == 1){  //RC
      //if(ebrake == false && value8 == 1){
      if(value8 == 1){
        if(value6 == 1) //Honda
          {
            if(throttle > 0)
            {
              analogWrite(9, 0);
              analogWrite(8, throttle);
            }
            
            else if(throttle < 0)
            {
              analogWrite(9, 250);
              analogWrite(8, abs(throttle));
            }
            else if (throttle == 0)
            {
              analogWrite(9, 0);
              analogWrite(8, 0);
            }
           
            return 1;
          }
         
      

        else if(value6 == 2) //Audi
      {
        analogWrite(7, value3);
        analogWrite(8, value3);
        return 2;
      }
        else if(value6 == 3) //BMW
      {
        analogWrite(7, value3);
        return 3;
      }
     }
     //else if(ebrake == true || value8 == 0){
     else if(value8 == 0){
      
      analogWrite(8, 0);
      analogWrite(9, 0);
      
      }
   }
  /*
   * value1 = Steering 
   * value2 = Right joy up/down (Empty)
   * value3 = Throttle
   * value4 = Left joy left/right (Empty)
   * value5 = RC/TO switch
   * value6 = Transfer box (FWD/AWD/RWD)
   * value8 = Safety switch
   */
  //RC mode
  
      
    if(value5 == 0){  //RC
      //if(ebrake == false && value8 == 1){
      if(value8 == 1){
        if(value6 == 1) //Honda
          {
            if(value3 > 0)
            {
              analogWrite(8, value3);
              analogWrite(9, 0);
            }
            
            if(value3 < 0)
            {
              analogWrite(9, 250);
              analogWrite(8, abs(value3));
            }
           
            return 1;
          }
         
      

        else if(value6 == 2) //Audi
      {
        analogWrite(7, value3);
        analogWrite(8, value3);
        return 2;
      }
        else if(value6 == 3) //BMW
      {
        analogWrite(7, value3);
        return 3;
      }
     }
     //else if(ebrake == true || value8 == 0){
     else if(value8 == 0){
      
      analogWrite(8, 0);
      analogWrite(9, 0);
      }
   }
  }
