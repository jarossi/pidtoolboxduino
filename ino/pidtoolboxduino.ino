#include <TimerThree.h>


#include <Timer.h>

#include <ThumbJoystick.h>

#include <MenuBackend.h>

#include <PID_AutoTune_v0.h>

#include <PID_v1.h>

#include <OneWire.h>
#include <DallasTemperature.h>

#include <Wire.h>
#include <LiquidCrystal.h>

#include <avr/eeprom.h>


// ThumbJoystick contants and variables
#define joystickSelPin A0
#define joystickXPin A1
#define joystickYPin A2
#define joystickXInvert true
#define joystickYInvert true
#define joystickThreshold 5

ThumbJoystick joystick(joystickSelPin, joystickXPin, joystickYPin, joystickXInvert, joystickYInvert);

// Connect via i2c, default address #0 (A0-A2 not jumpered)
LiquidCrystal lcd(0);
char lcdBlankLine[] = "                    ";

byte degree[8] = // define the degree symbol
{
  B00110,
  B01001,
  B01001,
  B00110,
  B00000,
  B00000,
  B00000,
  B00000
};




// Data wire is plugged into port 2 on the Arduino
#define oneWireBusPin 2
#define oneWireTemperaturePecision 12

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(oneWireBusPin);
// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

#define fanPin 10

struct config_t {
  // Pid
  double pidSetPoint;
  DeviceAddress pidInputSensor;
  byte pidOutputChannel;
  double pidKp;
  double pidKi;
  double pidKd;
  int pidDirection;
  boolean pidAMode;
  double pidADelta;
  double pidAKp;
  double pidAKi;
  double pidAKd;

  DeviceAddress customControlInputSensor;
  byte customControlOutputChannel;

  unsigned long SSRMinOn;
  unsigned long SSRMaxOff;
  unsigned long SSRPeriod;

  unsigned long relayMinOn;
  unsigned long relayMaxOff;
  unsigned long relayPeriod;

};

config_t configuration;


struct config_t_gen {
  int fanSpeed;
  int currentSettings;
};

# define maxCurrentSettings 10
config_t_gen configuration_general;


struct sensor_buf_t {
  DeviceAddress address;
  volatile float temp;
};

#define maxSensorCount 10
sensor_buf_t sensorBuffer[maxSensorCount];


Timer Timers;



#define customControlPotPin A4
#define customControlSwitchPin 7
#define customControlLedPin 9

double customControlInput;
byte customControlPot;


#define dimmerPin 11
#define SSRPin 13
#define relayPin 12


#define pidLedPin 8
#define pidSwitchPin 6
double pidSetPoint, pidInput, pidOutput;

//Specify the links and initial tuning parameters
PID pid(&pidInput, &pidOutput, &pidSetPoint, 1, 1, 1, DIRECT);

void printSerialData() {
  Serial.print(millis());
  Serial.print(",customControlPot:");
  Serial.println(customControlPot);

}


void delayUpdate(unsigned long d, void (*fUpdate)() ) {
  long startTime;

  startTime = millis();

  while ((millis() - startTime) < d) {
    fUpdate();
  }
  return;
}

boolean addressAreEqual(DeviceAddress d1, DeviceAddress d2) {
  for (uint8_t i = 0; i < 8; i++) {
    if (d1[i] != d2[i]) return false;
  }
  return true;
}


void lcdPrintIntSecondLine(long value) {
  lcd.setCursor(0, 1);
  lcd.print(lcdBlankLine);
  lcd.setCursor(0, 1);
  lcd.print(value);
}

void lcdPrintFloatSecondLine(double value) {
  lcd.setCursor(0, 1);
  lcd.print(lcdBlankLine);
  lcd.setCursor(0, 1);
  lcd.print(value);
}

void lcdPrintBooleanSecondLine(int value){
   lcd.setCursor(0, 1);
   lcd.print(lcdBlankLine);
   lcd.setCursor(0, 1);
   if (value == true){
     lcd.print("true");
   } else {
     lcd.print("false");
   }
  
}
  
void lcdPrintPidDirectionSecondLine(int value){
   lcd.setCursor(0, 1);
   lcd.print(lcdBlankLine);
   lcd.setCursor(0, 1);
   if (value == DIRECT){
     lcd.print("direct");
   } else {
     lcd.print("reverse");
   }
  
}
  
  
  

// Function to edit int
long editInt(long var, long minValue, long maxValue, long increment, int digits, void (*fdisplay)(long)) {
  long result = var;
  byte joystickreadD;
  int joystickreadA;
  long calincrement = 0;

  fdisplay(result);
  lcd.blink();
  delay(10);

  while (joystickreadD != THUMBJOYSTICK_SEL) {
    joystick.updateDigital();
    joystickreadD = joystick.readDigital();

    joystickreadA = joystick.getYAxisMaped(-digits, digits);

    if (joystickreadA != 0) {
      // Calculate increment
      calincrement = increment;
      for (int i = 1; i < abs(joystickreadA); i++) {
        calincrement = calincrement * 10;
      }

      if (joystickreadA < 0) result = constrain(result - calincrement, minValue, maxValue);
      if (joystickreadA > 0) result = constrain(result + calincrement, minValue, maxValue);
      fdisplay(result);
      delayUpdate(400, doUpdate);

    }
    doUpdate();
  }
  lcd.noBlink();
  return result;
}


double editFloat(double var, double minValue, double maxValue, double increment, int digits, void (*fdisplay)(double) ) {
  double result = var;
  byte joystickreadD;
  int joystickreadA;
  double calincrement = 0;

  fdisplay(result);
  lcd.blink();
  delay(10);

  while (joystickreadD != THUMBJOYSTICK_SEL) {
    joystick.updateDigital();
    joystickreadD = joystick.readDigital();

    joystickreadA = joystick.getYAxisMaped(-digits, digits);

    if (joystickreadA != 0) {
      // Calculate increment
      
      calincrement = increment;
      for (int i = 1; i < abs(joystickreadA); i++) {
        calincrement = calincrement * 10;
      }

      if (joystickreadA < 0) result = constrain(result - calincrement, minValue, maxValue);
      if (joystickreadA > 0) result = constrain(result + calincrement, minValue, maxValue);
      fdisplay(result);
      delayUpdate(400, doUpdate);

    }
    doUpdate();
  }
  lcd.noBlink();
  return result;
}

void lcdPrintTemp(float temp) {
  lcd.print(temp);
  if (! isnan(temp) ) {
    lcd.write(0);
    lcd.print("C");
    if (temp < 100) lcd.print(" ");
    if (temp < 10) lcd.print(" ");
  } else {
    lcd.print("     ");
  }

}

void lcdPrintSensorAddress (DeviceAddress d) {
  for (uint8_t i = 0; i < 8; i++) {
    // zero pad the address if necessary
    if ( d[i] < 16) lcd.print("0");
    lcd.print(d[i], HEX);
  }
}

void lcdPrintSensorAddressResTemp(long idx) {
  lcd.setCursor(0, 1);
  lcd.print(lcdBlankLine);
  lcd.setCursor(0, 1);
  lcdPrintSensorAddress(sensorBuffer[idx].address);
  lcd.setCursor(0, 2);
  lcd.print(lcdBlankLine);
  lcd.setCursor(0, 2);
  lcd.print("Resolution:");
  lcd.print(sensors.getResolution(sensorBuffer[idx].address));
  lcd.setCursor(0, 3);
  lcd.print(lcdBlankLine);
  lcd.setCursor(0, 3);
  lcdPrintTemp(sensorBuffer[idx].temp);

}

void editSensor(uint8_t* device) {
  int idx;
  sensors.begin();
  for (uint8_t i = 0; i < maxSensorCount; i++) {
    for (uint8_t  h = 0; h < 8; h++) {
      sensorBuffer[i].address[h] = 0;
    }
    sensorBuffer[i].temp = NAN;
  }
  sensors.setResolution(oneWireTemperaturePecision);
  sensors.setWaitForConversion(false);
  delayUpdate(750, doUpdate);
  idx = editInt(0, 0, sensors.getDeviceCount() - 1 , 1, 1, lcdPrintSensorAddressResTemp);
  sensors.getAddress(device, idx);;
  lcd.setCursor(0, 2);
  lcd.print(lcdBlankLine);
  lcd.setCursor(0, 3);
  lcd.print(lcdBlankLine);

}

#include "buildMenu.h"



void driveOutput(byte channel, byte value) {
  static unsigned long SSRStartTime;
  static unsigned long relayStartTime;
  unsigned long SSRMillisValue;
  unsigned long relayMillisValue;

  switch (channel) {
    case 1:
      analogWrite(dimmerPin, map(value, 0, 255, 5, 99));
      break;
    case 2:
      SSRMillisValue = map(value, 0, 255, 0, configuration.SSRPeriod);

      if (SSRMillisValue < configuration.SSRMinOn) {
        digitalWrite(SSRPin, LOW);
        break;
      }

      if (SSRMillisValue > configuration.SSRPeriod - configuration.SSRMaxOff) {
        digitalWrite(SSRPin, HIGH);
        break;
      }
      
      if (SSRStartTime > millis()) {
        SSRStartTime = millis();
      }

      if (millis() > (SSRStartTime + configuration.SSRPeriod) ) {
        SSRStartTime = millis();
      }
      
      if (millis() - SSRStartTime > configuration.SSRPeriod)
      { //time to shift the Relay Window
        SSRStartTime += configuration.SSRPeriod;
      }
      if (SSRMillisValue > millis() - SSRStartTime) {
        digitalWrite(SSRPin, HIGH);

      } else {
        digitalWrite(SSRPin, LOW);
      }
      break;
    case 3:
      relayMillisValue = map(value, 0, 255, 0, configuration.relayPeriod);

      if (relayMillisValue < configuration.relayMinOn) {
        digitalWrite(relayPin, LOW);
        break;
      }

      if (relayMillisValue > configuration.relayPeriod - configuration.relayMaxOff) {
        digitalWrite(relayPin, HIGH);
        break;
      }
      
      if (relayStartTime > millis()) {
        relayStartTime = millis();
      }

      if (millis() > (relayStartTime + configuration.relayPeriod) ) {
        relayStartTime = millis();
      }
      
      if (millis() - relayStartTime > configuration.relayPeriod)
      { //time to shift the Relay Window
        relayStartTime += configuration.relayPeriod;
      }
      if (relayMillisValue > millis() - relayStartTime) {
        digitalWrite(relayPin, HIGH);

      } else {
        digitalWrite(relayPin, LOW);
      }
      break;
  }

}

void driveLed(byte pin, byte channel, byte value) {
  boolean val;
  switch (channel) {
    case 1:
      analogWrite(pin, value);
      break;
    case 2:
      //val = digitalRead(SSRPin);
      digitalWrite(pin, digitalRead(SSRPin));
      break;
    case 3:
      //val = digitalRead(relayPin);
      digitalWrite(pin, digitalRead(relayPin));
      break;
  }

}


void doOftenUpdate() {

  // debug
  if (digitalRead(pidSwitchPin) == LOW && !(isnan(pidInput))  &&  !(configuration.pidOutputChannel == configuration.customControlOutputChannel && digitalRead(customControlSwitchPin) == LOW)  ) {
    pid.SetMode(AUTOMATIC);
    driveOutput(configuration.pidOutputChannel, pidOutput);
    driveLed(pidLedPin, configuration.pidOutputChannel, pidOutput);
  } else if( ! (configuration.pidOutputChannel == configuration.customControlOutputChannel && digitalRead(customControlSwitchPin) == LOW)) {
    pid.SetMode(MANUAL);
    driveOutput(configuration.pidOutputChannel, 0);
    driveLed(pidLedPin, configuration.pidOutputChannel, 0);
  }

  if (digitalRead(customControlSwitchPin) == LOW) {
    driveOutput(configuration.customControlOutputChannel, customControlPot);
    driveLed(customControlLedPin, configuration.customControlOutputChannel, customControlPot);
  } else {
    driveOutput(configuration.customControlOutputChannel, 0);
    driveLed(customControlLedPin, configuration.customControlOutputChannel, 0);
  }

  for (uint8_t i = 1; i < 4; i++) {
    if (configuration.pidOutputChannel == i || configuration.customControlOutputChannel == i ) {
      continue;
    }
    driveOutput(i, 0);
  }
  
  pid.Compute();
  
}


void lcdPrintDash() {
  if (menu.getCurrent().isEqual(mHomeScreen)) {
    //lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("PID      Settings:");
    lcd.print(configuration_general.currentSettings);
    if ( configuration_general.currentSettings < 10) lcd.print(" ");

    doOftenUpdate();

    //pid

    lcd.setCursor(0, 1);
    lcd.print("PS:");
    lcdPrintTemp(pidSetPoint);
    lcd.setCursor(13, 1);
    lcd.print("MO:");

    doOftenUpdate();

    lcd.setCursor(0, 2);
    lcd.print("PI:");
    lcdPrintTemp(pidInput);
    lcd.setCursor(13, 2);
    lcd.print("PO:");
    lcd.print((byte)pidOutput);
    if ((byte)pidOutput < 100) lcd.print(" ");
    if ((byte)pidOutput < 10) lcd.print(" ");

    doOftenUpdate();

    // Custom Control
    lcd.setCursor(0, 3);
    lcd.print("CCI:");
    lcdPrintTemp(customControlInput);
    lcd.setCursor(13, 3);
    lcd.print("CCP:");
    lcd.print(customControlPot);
    if (customControlPot < 100) lcd.print(" ");
    if (customControlPot < 10) lcd.print(" ");
  }

}


void updateSensorBuffer() {
  int deviceCount=sensors.getDeviceCount();
  for (uint8_t i = 0; i < deviceCount; i++) {
    sensors.getAddress(sensorBuffer[i].address, i);
    sensorBuffer[i].temp = sensors.getTempCByIndex(i);
    doOftenUpdate();

  }
  sensors.requestTemperatures();
  for (uint8_t i = 0; i < deviceCount; i++) {
    if ( addressAreEqual(sensorBuffer[i].address, configuration.customControlInputSensor) ) customControlInput = sensorBuffer[i].temp;
    if ( addressAreEqual(sensorBuffer[i].address, configuration.pidInputSensor) ) pidInput = sensorBuffer[i].temp;
  }
}




void doUpdate() {
  customControlPot = map( analogRead(customControlPotPin), 0 , 1023, 0, 255);

  double gap = abs(pidSetPoint - pidInput); //distance away from setpoint
  if ( configuration.pidAMode && gap > configuration.pidADelta) {
    pid.SetTunings(configuration.pidAKp, configuration.pidAKi, configuration.pidAKd);
  }
  else
  {
    //we're far from setpoint, use aggressive tuning parameters
    pid.SetTunings(configuration.pidKp, configuration.pidKi, configuration.pidKd);
  }

  doOftenUpdate();
  
  Timers.update();


}

void setup() {

  // set up the LCD's number of rows and columns:
  lcd.begin(20, 4);

  // Print a message to the LCD.
  lcd.setBacklight(HIGH);
  delay(100);
  lcd.print("Initializing");
  lcd.createChar(0, degree);

  // read configuration from eeprom
  /*
  configuration_general.fanSpeed=15;
  configuration_general.currentSettings=1;
  eeprom_write_block((const void*)&configuration_general, (void*)0, sizeof(configuration));
  */

  eeprom_read_block((void*)&configuration_general, (void*)0, sizeof(configuration));

  /*
  configuration.pidSetPoint=0;
  sensors.getAddress(configuration.pidInputSensor, 0);
  configuration.pidOuputChannel=1;
  configuration.pidKp=1;
  configuration.pidKi=1;
  configuration.pidKd=1;
  configuration.pidDirection=DIRECT;
  configuration.pidAMode=false;
  configuration.pidADelta=0;
  configuration.pidAKp=1;
  configuration.pidAKi=1;
  configuration.pidAKd=1;

  sensors.getAddress(configuration.customControlInputSensor, 1);
  configuration.customControlOutputChannel=2;

  configuration.SSRMinOn=100;
  configuration.SSRMaxOff=100;
  configuration.SSRPeriod=1000;

  configuration.relayMinOn=1000;
  configuration.relayMaxOff=1000;
  configuration.relayPeriod=5000;

  for (uint8_t i=0; i<maxCurrentSettings; i++) {
    eeprom_write_block((const void*)&configuration, (void*)(  (i+1) * sizeof(configuration)  ), sizeof(configuration));

  }
  */
  eeprom_read_block((void*)&configuration, (void*)( configuration_general.currentSettings * sizeof(configuration)  ), sizeof(configuration));

  //Joystick
  joystick.setZeros();


  // Start up sensors
  sensors.begin();
  sensors.setResolution(oneWireTemperaturePecision);
  sensors.setWaitForConversion(false);
  sensors.requestTemperatures();
  for (uint8_t i = 0; i < maxSensorCount; i++) {
    for (uint8_t  h = 0; h < 8; h++) {
      sensorBuffer[i].address[h] = 0;
    }
    sensorBuffer[i].temp = NAN;
  }



  customControlInput = 0;
  customControlPot = 0;
  pinMode(customControlPotPin, INPUT);
  pinMode(customControlSwitchPin, INPUT_PULLUP);
  pinMode(customControlLedPin, OUTPUT);
  digitalWrite(customControlLedPin, HIGH);


  //init PID
  pidInput = 0;
  pidSetPoint = configuration.pidSetPoint;
  pidOutput = 0;
  pinMode(pidSwitchPin, INPUT_PULLUP);
  pinMode(pidLedPin, OUTPUT);
  digitalWrite(pidLedPin, HIGH);
  //pidkp, etc.
  pid.SetMode(MANUAL);
  pid.SetTunings(configuration.pidKp, configuration.pidKi, configuration.pidKd);


  pinMode (dimmerPin, OUTPUT);
  pinMode (SSRPin, OUTPUT);
  pinMode (relayPin, OUTPUT);


  // Init the fan
  pinMode(fanPin, OUTPUT);
  analogWrite(fanPin, 255);



  delay(3000);
  analogWrite(fanPin, configuration_general.fanSpeed);
  digitalWrite(customControlLedPin, LOW);
  digitalWrite(pidLedPin, LOW);

  // Init the menu
  menuSetup();
  menu.moveDown();
  lcdPrintDash();

  Serial.begin(9600);

  if (configuration.customControlInputSensor == configuration.pidInputSensor) {
    Serial.println("NO");
  }

  // We init the timers
  //Timers.every(10, doOftenUpdate);
  Timers.every(500, printSerialData);
  Timers.every(750, updateSensorBuffer);
  Timers.every(300, lcdPrintDash);



}

void loop() {
  // put your main code here, to run repeatedly:

  joystick.updateDigital();
  byte joystickread = joystick.readDigital();
  if (joystickread != THUMBJOYSTICK_NULL) {
    switch (joystickread) {
      case THUMBJOYSTICK_UP: menu.moveUp(); break;
      case THUMBJOYSTICK_DOWN: menu.moveDown(); break;
      case THUMBJOYSTICK_RIGHT: menu.moveRight(); break;
      case THUMBJOYSTICK_LEFT: menu.moveLeft(); break;
      case THUMBJOYSTICK_SEL: menu.use(); break;
    }
  }

  doUpdate();

}
