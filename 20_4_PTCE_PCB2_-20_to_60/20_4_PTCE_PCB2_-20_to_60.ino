//THIS PROGRAM WAS CHANGED flipped the HVPS fault and made limit 8 to avoid fault
#include <Adafruit_MAX31856.h>
#include <Wire.h> 
#include <LCD.h>
#include <LiquidCrystal_I2C.h>

#define I2C_ADDR 0x27
#define BACKLIGHT_PIN 3
#define En_pin 2
#define Rw_pin 1
#define Rs_pin 0
#define D4_pin 4
#define D5_pin 5
#define D6_pin 6
#define D7_pin 7
#define VIN A5 //  A0 defined for Voltage INPUT (V in)

LiquidCrystal_I2C lcd(I2C_ADDR,En_pin,Rw_pin,Rs_pin,D4_pin,D5_pin,D6_pin,D7_pin);
Adafruit_MAX31856 maxthermo = Adafruit_MAX31856(10, 11, 2, 13); //4,5,6,7,8,12 used by relay shield

const int          alarmPin = A0;           //ALARM PIN FROM HVPS changed from A0
const double       alarmThresh = 8;       //SET POINT for ALARM threshold

double             val = 0.0;               //Initialize ALARM READ

const double       tHighThresh = 60.0;      //95 SET MAX TEMP IN THE CYCLE
const double       rampUpThresh = -16.5;     //-36 SET MIN UP 'on' IN THE CYCLE
const double       tLowThresh = -20;       //-20 SET MIN TEMP IN THE CYCLE

const double       rampDownThresh = 57.0;   //SET MIN DOWN 'off' in THE CYCLE

const double       tSlopeThresh = 0.1;      //
double             tSlope = -1.0;           //SET INITIAL SLOPE - to start OFF

const int          temp_size = 50; 
double             lastTemps[temp_size];    //ARRAY OF  SIZE determined by 'temp_size'

const int relayPins[] = {4,7,8,12};         //DECLARE THE PINS ON THE RELAY SHIELD
const int relayUse[] = {1,1,1,1};           //DECLARE PINS USED FROM THE RELAY SHIELD


//////////*************************************VOID SETUP*****************************************/////////

void setup() {
  
  pinMode(9,OUTPUT);                        //  STATUS LED: RED
  pinMode(3,OUTPUT);                        // GREEN LED FOR STATUS: OKAY
  digitalWrite(3,LOW);                     //SET GREEN 'ON'
  digitalWrite(9,LOW);                      //SET  RED 'OFF'

  Serial.begin(9600);
  Serial.println("STEP1: INITIALIZE ALL RELAYS TO LOW STATE NC");
  
  for (int i=0; i<4;  i++) {
    pinMode(relayPins[i], OUTPUT);
    digitalWrite(relayPins[i], HIGH);      //INITIALIZE TO START WITH HVPS 'OFF' untill CHECK
  }
  Serial.println("");

  Serial.println("STEP2: INITIALIZE INDEX TEMP IN ARRAY FOR SLOPE CALC");
   for (int i = 0; i < temp_size; i++) {
     lastTemps[i] = 0;                    //THIS SETS ALL TEMPERATURE TO 150 FOR THE ARRAY 
 }
 
 Serial.println("");
 Serial.println("STEP 3: INITIALIZING MAX31856 thermocouple..");
 Serial.println("        Running TEST...");
 maxthermo.begin();
 maxthermo.setThermocoupleType(MAX31856_TCTYPE_K);
 Serial.println("        Test SUCCESFUL!!");
 Serial.print("        Using Thermocouple type: ");
  switch (maxthermo.getThermocoupleType() ) {
    case MAX31856_TCTYPE_B: Serial.println("B Type"); break;
    case MAX31856_TCTYPE_E: Serial.println("E Type"); break;
    case MAX31856_TCTYPE_J: Serial.println("J Type"); break;
    case MAX31856_TCTYPE_K: Serial.println("K Type"); break;
    case MAX31856_TCTYPE_N: Serial.println("N Type"); break;
    case MAX31856_TCTYPE_R: Serial.println("R Type"); break;
    case MAX31856_TCTYPE_S: Serial.println("S Type"); break;
    case MAX31856_TCTYPE_T: Serial.println("T Type"); break;
    case MAX31856_VMODE_G8: Serial.println("Voltage x8 Gain mode"); break;
    case MAX31856_VMODE_G32: Serial.println("Voltage x8 Gain mode"); break;
    default: Serial.println("Unknown"); break;
  }
 Serial.println(" ");
 lcd.clear(); 
 lcd.begin(20,4);  //CHANGE 16,2 TO 20,4 to change display 
 lcd.setBacklightPin(BACKLIGHT_PIN,POSITIVE);
 lcd.setBacklight(HIGH);
  
}


//************FUNCTION GET THE SLOPE****************//

double getSlope(double temps[temp_size]) {
    double dT[temp_size-1];
    double slopeOut = 0.0;
    for (int i = 0; i < temp_size-1; i++) {
      dT[i] = temps[i + 1] - temps[i];
      //Serial.println(dT[i]);
      slopeOut = slopeOut + dT[i];
    }
    //slopeOut = slopeOut / (temp_size-1);
    // Slope is arbitrary.  Divide by 5 instead of array size too keep the value large for arithmetic
    slopeOut = slopeOut / 50.0;
    return slopeOut;
}


//////////**************************************VOID LOOP ****************************************/////////

void loop() {
  double c = maxthermo.readThermocoupleTemperature(); //initializing constant fo use TCtemp
  double TCtemp= c; 
  val = analogRead(alarmPin);
  val = map(val, 0, 1023, 0, 5);
  int time = millis()/1000; // Sec 
  
  Serial.println("STEP4: Take a sample reading of Thermocouple and JT");
  Serial.print("    Cold Junction Temp:   "); 
  Serial.println(maxthermo.readCJTemperature());
  Serial.print("    Thermocouple Temp:    "); 
  Serial.println(maxthermo.readThermocoupleTemperature());
  Serial.println();

  lcd.clear();
  lcd.setCursor(0,1);
  lcd.print("TEMPERATURE:");
  lcd.print(TCtemp);
  lcd.print(" 'C");
  lcd.setCursor(0,2);
  lcd.print("SLOPE:      ");
  lcd.print(tSlope);
  
/////////////////////////////////HVPS ALARM//////////////////////////////////////////////
if (val < alarmThresh) {
        Serial.print("HVPS ALARM: OKAY ");  //SETTING LOW AT THE END
        Serial.println(val);
        lcd.setCursor(0,0);
        lcd.print(" HVPS ALARM: 'OKAY '");
      }
else {
        for (int i=0; i<4;  i++){
          if(relayUse[i]==1) {
            digitalWrite(relayPins[i],HIGH);
            }
         }
        Serial.println("HVPS ALARM STATUS: FAULT! Value: ");  //SETTING HIGH RIGHT AWAY
        Serial.print(val);
        lcd.setCursor(0,0);
        lcd.print(" HVPS ALARM: FAULT ");
      }
//////////////////////////////////RAMP UP ///////////////////////////////////////////// 

 if ( (TCtemp>tLowThresh) and (TCtemp>=rampUpThresh) and ( TCtemp<(tHighThresh*1.05))and(tSlope>=-.01) ) {
   Serial.println("RAMP UP:'System on'");  //TURN ON AT THE END
  // lcd.clear();
  lcd.setCursor(0,3);
  lcd.print("RAMP UP: SYSTEM ON ");
   }
//////////////////////////////////RAMP DOWN ///////////////////////////////////////////// 
 else if ( (TCtemp>tLowThresh) and (TCtemp>rampDownThresh) and ( TCtemp<(tHighThresh*1.05))and (tSlope<.03) ){
  Serial.println("RAMP DOWN... 'HVPS on'");  //TURN ON AT THE END
  lcd.setCursor(0,3);
  lcd.print("Ramp DOWN:SYSTEM ON "); 
 }
 else { 
        for (int i=0; i<4;  i++){
            if(relayUse[i]==1) {
                digitalWrite(relayPins[i],HIGH);} //IMMEDIATELY TURN OFF
             }     
        Serial.println("RAMPING UP :  BELOW rampUpThresh or Temp FAULT"); 
        lcd.setCursor(0,3);
        lcd.print("    'system off' ");  
      }

 /////////////////////////////////FINAL EXECUTION/////////////////////////////////////////////////////    
if( ((val < alarmThresh) and  ( ((TCtemp>tLowThresh) and (TCtemp>rampUpThresh) and (TCtemp<(tHighThresh*1.05)))and (tSlope>.01) )  ) or
    
    ((val > alarmThresh) and  ( ((TCtemp>tLowThresh) and (TCtemp>rampDownThresh) and ( TCtemp<(tHighThresh*1.05)))and((tSlope<=.03) ))  )
  ){
      for (int i=0; i<4;  i++){
              if(relayUse[i]==1) {
                digitalWrite(relayPins[i],LOW);
                }
             }
      Serial.println("RE-STARTING LOOP CHECK...NO FAULTS FOUND");
      digitalWrite(3,HIGH);                     //SET GREEN 'ON'
      digitalWrite(9,LOW);                      //SET  RED 'OFF'
  }
  
else{
  Serial.println("Relay CAN NOT be RESETTED due FAULTs OR Temperature THRESHOLD LIMITS !!!!" );
  digitalWrite(3,LOW);                     //SET GREEN 'ON'
  digitalWrite(9,HIGH);                      //SET  RED 'ON'
}

 //////////////////////// 
  // Check and print any faults
  uint8_t fault = maxthermo.readFault();
  if (fault) {
    if (fault & MAX31856_FAULT_CJRANGE) Serial.println("Cold Junction Range Fault");
    if (fault & MAX31856_FAULT_TCRANGE) Serial.println("Thermocouple Range Fault");
    if (fault & MAX31856_FAULT_CJHIGH)  Serial.println("Cold Junction High Fault");
    if (fault & MAX31856_FAULT_CJLOW)   Serial.println("Cold Junction Low Fault");
    if (fault & MAX31856_FAULT_TCHIGH)  Serial.println("Thermocouple High Fault");
    if (fault & MAX31856_FAULT_TCLOW)   Serial.println("Thermocouple Low Fault");
    if (fault & MAX31856_FAULT_OVUV)    Serial.println("Over/Under Voltage Fault");
    if (fault & MAX31856_FAULT_OPEN)    Serial.println("Thermocouple Open Fault");
  }
  for (int i = 0; i < temp_size-1; i++) {
    lastTemps[i] = lastTemps[i + 1];
    Serial.println(lastTemps[i]);
  }
  lastTemps[temp_size-1] = c;
  Serial.println(lastTemps[temp_size-1]);
  tSlope = getSlope(lastTemps);
  Serial.print("slope: ");
  Serial.println(tSlope);
  
  delay(100);
}
