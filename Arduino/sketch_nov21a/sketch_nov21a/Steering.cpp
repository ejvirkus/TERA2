#include "Steering.h"
#include "Arduino.h"
#define DIR 44      //suuna määramise pin
#define PUL 40      //steering pulse pin
#define STEERING_CYCLE 60 //keeramise kiirus 20
#define DELAY 50
/*
int mapping(int a1, int a2, int b1, int b2, int input){
  int inValNorm = input - a1;
  int aUpperNorm = a2 - a1;
  int normPos = inValNorm / aUpperNorm;

  int bUpperNorm = b2 - b1;
  int bValNorm = normPos * bUpperNorm;
  int outVal = b1 + bValNorm;

  return outVal;
}
*/

Steering::Steering()
{
  pinMode(DIR, OUTPUT);
  pinMode(PUL, OUTPUT);
}

 int Steering::Left_Right(int value1, int value5, int encoder, int wheel)
{
/*
  if (value5 == 1)
  {
    encoder = encoder / 10;

//---------------------------------TELEOPIGA TAGASI KESKELE-----------------------------------------------------------------------------------------------------

    int difference = encoder - 267;
    //Serial.println(difference);
    if(digitalRead(TO_RIGHT) == 0 && digitalRead(TO_LEFT) == 0){

      if(difference < 0 &&  abs(difference) > 5){               //PAREMALE
      digitalWrite(DIR, LOW);   //PAREMALE

      for (int i = 0; i < STEERING_CYCLE; i++){    //ühe tsükli pikkus, mida suurem seda pikem ring
        //these 4 lines result in 1 step:
        digitalWrite(PUL, HIGH); //PIN 6 = stepPin
        delayMicroseconds(DELAY);
        digitalWrite(PUL, LOW);
        delayMicroseconds(DELAY);
      }
      }
      if(difference > 0 &&  abs(difference) > 5){               //VASAKULE
      digitalWrite(DIR, HIGH);   //VASAKULE

      for (int i = 0; i < STEERING_CYCLE; i++){    //ühe tsükli pikkus, mida suurem seda pikem ring
        //these 4 lines result in 1 step:
        digitalWrite(PUL, HIGH); //PIN 6 = stepPin
        delayMicroseconds(DELAY);
        digitalWrite(PUL, LOW);
        delayMicroseconds(DELAY);
      }
      }

    }
    */
//-----------------------TELEOP KEERAMINE------------------------------------------------------------------------------------------------------------------
//Steering range = 0(vasak) - 655
//Steering centre = 328
//Encoder values = 365 - 170 (195)

if(value5 == 1){
    encoder = encoder / 10;
    //Serial.print(encoder);
    //Serial.print("\n");
    int realwheel = 268;
    
    //realwheel = map(wheel, 0, 655, 365, 170);
    realwheel = map(wheel, 0, 655, 170, 365);
    
      
    //mapping(0, 655, 365, 170, wheel);
    //Serial.println(realwheel);
    int difference = encoder - realwheel;
    //Serial.println(difference);
    if(realwheel > 169 && realwheel < 366){
  
      if(difference < 0 &&  abs(difference) > 8){
        digitalWrite(DIR, LOW);   //PAREMALE
  
        for (int i = 0; i < STEERING_CYCLE; i++){    //ühe tsükli pikkus, mida suurem seda pikem ring
          //these 4 lines result in 1 step:
          digitalWrite(PUL, HIGH); //PIN 6 = stepPin
          delayMicroseconds(DELAY);
          digitalWrite(PUL, LOW);
          delayMicroseconds(DELAY);
        }
      }
      if(difference > 0 && abs(difference) > 8){
        digitalWrite(DIR, HIGH);   //VASAKULE
  
        for (int i = 0; i < STEERING_CYCLE; i++){    //ühe tsükli pikkus, mida suurem seda pikem ring
          //these 4 lines result in 1 step:
          digitalWrite(PUL, HIGH); //PIN 6 = stepPin
          delayMicroseconds(DELAY);
          digitalWrite(PUL, LOW);
          delayMicroseconds(DELAY);
        }
      }
    }
  }
    //TULEB MUUTA PINID MILLELT SAAME INFOT KAS KEERAME VÕI EI (TO_RIGHT & TO_LEFT)
/*
    if(digitalRead(TO_RIGHT) == 1){
      digitalWrite(DIR, LOW);   //PAREMALE

      for (int i = 0; i < STEERING_CYCLE; i++){    //ühe tsükli pikkus, mida suurem seda pikem ring
        //these 4 lines result in 1 step:
        digitalWrite(PUL, HIGH); //PIN 6 = stepPin
        delayMicroseconds(DELAY);
        digitalWrite(PUL, LOW);
        delayMicroseconds(DELAY);
      }
    }
    if(digitalRead(TO_LEFT) == 1){
      digitalWrite(DIR, HIGH);   //VASAKULE

      for (int i = 0; i < STEERING_CYCLE; i++){    //ühe tsükli pikkus, mida suurem seda pikem ring
        //these 4 lines result in 1 step:
        digitalWrite(PUL, HIGH); //PIN 6 = stepPin
        delayMicroseconds(DELAY);
        digitalWrite(PUL, LOW);
        delayMicroseconds(DELAY);
      }
    }
  }
*/
//----------------------------RC KEERAMINE----------------------------------------------------------------
//Vahemik = 365-170
  else if(value5 == 0)
  {
    
    encoder = encoder / 10;
    //Serial.print(encoder);
    //Serial.print("\n");

    int difference = encoder - value1;
    //Serial.println(difference);

    if(difference < 0 &&  abs(difference) > 10){
      digitalWrite(DIR, LOW);   //PAREMALE

      for (int i = 0; i < STEERING_CYCLE; i++){    //ühe tsükli pikkus, mida suurem seda pikem ring
        //these 4 lines result in 1 step:
        digitalWrite(PUL, HIGH); //PIN 6 = stepPin
        delayMicroseconds(DELAY);
        digitalWrite(PUL, LOW);
        delayMicroseconds(DELAY);
      }
    }
    if(difference > 0 && abs(difference) > 10){
      digitalWrite(DIR, HIGH);   //VASAKULE

      for (int i = 0; i < STEERING_CYCLE; i++){    //ühe tsükli pikkus, mida suurem seda pikem ring
        //these 4 lines result in 1 step:
        digitalWrite(PUL, HIGH); //PIN 6 = stepPin
        delayMicroseconds(DELAY);
        digitalWrite(PUL, LOW);
        delayMicroseconds(DELAY);
      }
    }
  }
}
