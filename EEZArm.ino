#include <movingAvg.h>
#include <Ps3Controller.h>
#include <ESP32Servo.h>
#include <EEPROM.h>

#define BASE_PIN 21
#define UPPER_PIN 22
#define LOWER_PIN 23

Servo base;
Servo upper;
Servo lower;

int basePos = 1500;
int basePosSmoothed;
int basePosPrev;
int upperPos = 1500;
int upperPosSmoothed;
int upperPosPrev;
int lowerPos = 1500;
int lowerPosSmoothed;
int lowerPosPrev;

int baseMin = 0;
int baseMax = 0;
int lowerMin = 0;
int lowerMax = 0;
int upperMax = 0;
int upperMin = 0;

movingAvg leftYAvg(10);
movingAvg leftXAvg(10);
movingAvg rightYAvg(10);

void notify(){
   //---------------- Analog stick value events ---------------
   if( Ps3.event.button_down.cross ){
    if(basePos < 1499){
      baseMin = basePos;
      EEPROM.write(0, baseMin);
    } else {
      baseMax = basePos;
      EEPROM.write(1, baseMax);
    }
    EEPROM.commit();
   }

   if( Ps3.event.button_down.square ){
    if(lowerPos < 1499){
      lowerMin = lowerPos;
      EEPROM.write(2, lowerMin);
    } else {
      lowerMax = lowerPos;
      EEPROM.write(3, lowerMax);
    }
    EEPROM.commit();
   }

   if( Ps3.event.button_down.circle ){
    if(upperPos < 1499){
      upperMin = upperPos;
      EEPROM.write(4, upperMin);
    } else {
      upperMax = upperPos;
      EEPROM.write(5, upperMax);
    }
    EEPROM.commit();
   }


   if( Ps3.event.button_down.select ){
    EEPROM.write(0, 0);
    EEPROM.write(1, 0);
    EEPROM.write(2, 0);
    EEPROM.write(3, 0);
    EEPROM.write(4, 0);
    EEPROM.write(5, 0);

    EEPROM.commit();
    
    baseMax = EEPROM.read(0);
    baseMin = EEPROM.read(1);
    lowerMax = EEPROM.read(2);
    lowerMin = EEPROM.read(3);
    upperMax = EEPROM.read(4);
    upperMin = EEPROM.read(5);
    baseMin, baseMax, lowerMin, lowerMax, upperMin, upperMax= 0;
   }
}

void onConnect(){
    Serial.println("Connected.");
}

void setup() {
  Serial.begin(9600);

  //Load Memory
  EEPROM.begin(10);
  baseMax = EEPROM.read(0);
  baseMin = EEPROM.read(1);
  lowerMax = EEPROM.read(2);
  lowerMin = EEPROM.read(3);
  upperMax = EEPROM.read(4);
  upperMin = EEPROM.read(5);

  //PS3
  Ps3.attach(notify);
  Ps3.attachOnConnect(onConnect);
  Ps3.begin("00:12:34:56:78:9b");
  
  // Allow allocation of all timers
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  base.setPeriodHertz(50);
  upper.setPeriodHertz(50);
  lower.setPeriodHertz(50);
  
  base.attach(BASE_PIN, 500, 2500);
  upper.attach(UPPER_PIN, 500, 2500);
  lower.attach(LOWER_PIN, 500, 2500);

  leftYAvg.begin();
  leftXAvg.begin();
  rightYAvg.begin();
}

void loop() {
  if(!Ps3.isConnected()){
    return;  
    Serial.println(baseMin);
  }

   int leftX = Ps3.data.analog.stick.lx;
   int smoothedLeftX = leftXAvg.reading(leftX);
   if(leftX < -1 ){
    basePos = map(smoothedLeftX, -127, 0, baseMin ? baseMin : 500, (baseMin && baseMax) ? ((baseMax-baseMin)/2) : 1500);
    basePosSmoothed = (basePos * 0.05) + (basePosPrev * 0.95);
    basePosPrev = basePos;
   } else if(leftX > 1){
    basePos = map(smoothedLeftX, 0, 127, (baseMin && baseMax) ? ((baseMax-baseMin)/2) : 1500, baseMax ? baseMax : 2500);
    basePosSmoothed = (basePos * 0.05) + (basePosPrev * 0.95);
    basePosPrev = basePos;
   } else {
    basePos = (baseMin && baseMax) ? ((baseMax-baseMin)/2) : 1500;
     basePosSmoothed = (basePos * 0.05) + (basePosPrev * 0.95);
     basePosPrev = basePos;
   }
  
   int leftY = Ps3.data.analog.stick.ly;
   int smoothedLeftY = leftYAvg.reading(leftY);
   if(leftY < -1){
    lowerPos =  map(smoothedLeftY, -127, 0, lowerMin ? lowerMin : 500, (lowerMin && lowerMax) ? ((lowerMax-lowerMin)/2) : 1500);
    lowerPosSmoothed = (lowerPos * 0.05) + (basePosPrev * 0.95);
    lowerPosPrev = lowerPos;
   } else if(leftY > 1){
    lowerPos =  map(smoothedLeftY, 0, 127, (lowerMin && lowerMax) ? ((lowerMax-lowerMin)/2) : 1500, lowerMax ? lowerMax : 2500);
    lowerPosSmoothed = (lowerPos * 0.05) + (basePosPrev * 0.95);
    lowerPosPrev = lowerPos;
   } else {
    lowerPos = (lowerMin && lowerMax) ? ((lowerMax-lowerMin)/2) : 1500;
    lowerPosSmoothed = (lowerPos * 0.05) + (lowerPosPrev * 0.95);
    lowerPosPrev = lowerPos;
   }

   int rightY = Ps3.data.analog.stick.ry;
   int smoothedRightY = rightYAvg.reading(rightY);
   if(rightY < -1){
    upperPos =  map(smoothedRightY, -127, 0, upperMin ? upperMin : 500, (upperMin && upperMax) ? ((upperMax-upperMax)/2) : 1500);
    upperPosSmoothed = (upperPos * 0.05) + (upperPosPrev * 0.95);
    upperPosPrev = upperPos;
   } else if(rightY > 1){
    upperPos =  map(smoothedRightY, 0, 127, (upperMin && upperMax) ? ((upperMax-upperMax)/2) : 1500, upperMax ? upperMax : 2500);
    upperPosSmoothed = (upperPos * 0.05) + (upperPosPrev * 0.95);
    upperPosPrev = upperPos;
   } else {
    upperPos = (upperMin && upperMax) ? ((upperMax-upperMax)/2) : 1500;
    upperPosSmoothed = (upperPos * 0.05) + (upperPosPrev * 0.95);
    upperPosPrev = upperPos;
   }

  upper.writeMicroseconds(upperPosSmoothed);
  base.writeMicroseconds(basePosSmoothed);
  lower.writeMicroseconds(lowerPosSmoothed);
  
  Serial.print("Base:");
  Serial.print(base.readMicroseconds());
  Serial.print(" ");
  Serial.print("Lower:");
  Serial.print(lower.readMicroseconds());
  Serial.print(" ");
  Serial.print("Upper:");
  Serial.println(upper.readMicroseconds());
}
