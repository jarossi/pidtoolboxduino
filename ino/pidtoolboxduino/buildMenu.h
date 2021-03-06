


void menuChangeEvent(MenuChangeEvent changed);
void menuUseEvent(MenuUseEvent used);

//this controls the menu backend and the event generation
MenuBackend menu = MenuBackend(menuUseEvent,menuChangeEvent);
  //beneath is list of menu items needed to build the menu
  MenuItem mHomeScreen = MenuItem(menu, "HomeScreen", 1);
    MenuItem mPidSettings = MenuItem(menu, "PID Settings", 2);
      MenuItem mPidSetPoint = MenuItem(menu, "PID Set Point", 3);
      MenuItem mPidInputSensor = MenuItem(menu, "PID Input Sensor", 3);
      MenuItem mPidOutputChannel = MenuItem(menu, "PID Output Channel", 3);
      MenuItem mPidKp = MenuItem(menu, "PID Kp", 3);
      MenuItem mPidKi = MenuItem(menu, "PID Ki", 3);
      MenuItem mPidKd = MenuItem(menu, "PID Kd", 3);
      MenuItem mPidDirection = MenuItem(menu, "PID Direction", 3);
      MenuItem mPidAMode = MenuItem(menu, "PID Agressive Mode", 3);
      MenuItem mPidADelta = MenuItem(menu, "PID Agressive Delta", 3);
      MenuItem mPidAKp = MenuItem(menu, "PID Agressive Kp", 3);
      MenuItem mPidAKi = MenuItem(menu, "PID Agressive Ki", 3);
      MenuItem mPidAKd = MenuItem(menu, "PID Agressive Kd", 3);
      MenuItem mPidSampleTime = MenuItem(menu, "PID Sample Time", 3);
      
    MenuItem mPidAutoTuneSettings = MenuItem(menu, "PID Auto Tuner", 2);
      MenuItem mPidATuneInputNoise = MenuItem(menu, "PID AT In Noise Band", 3);
      MenuItem mPidATuneOutputStep = MenuItem(menu, "PID AT Out Step", 3);
      MenuItem mPidATuneLookBack = MenuItem(menu,   "PID AT Look Back Sec", 3);
      MenuItem mPidATuneControlType = MenuItem(menu,"PID AT Control Type", 3);
      MenuItem mPidATuneEnable = MenuItem(menu,"PID AT Enable", 3);

    MenuItem mCustomControlSettings = MenuItem(menu, "Custom Control", 2);
      MenuItem mCustomControlInputSensor = MenuItem(menu, "CC Input Sensor", 3);
      MenuItem mCustomControlOutputChannel = MenuItem(menu, "CC Output Channel", 3);
    
    MenuItem mSSRSettings = MenuItem(menu, "SSR Settings", 2);
      MenuItem mSSRMinOn = MenuItem(menu, "SSR Min On Time", 3);
      MenuItem mSSRMaxOff = MenuItem(menu, "SSR Man Off Time", 3);
      MenuItem mSSRPeriod = MenuItem(menu, "SSR Period", 3);
    
    
    MenuItem mRelaySettings = MenuItem(menu, "Relay Settings", 2);
      MenuItem mRelayMinOn = MenuItem(menu, "Relay Min On Time", 3);
      MenuItem mRelayMaxOff = MenuItem(menu, "Relay Max Off Time", 3);
      MenuItem mRelayPeriod = MenuItem(menu, "Relay Period", 3);
            
    MenuItem mGeneralSettings =  MenuItem(menu, "General Settings", 2);
      MenuItem mSaveCurrentSettings = MenuItem(menu, "Save Settings", 3);
      MenuItem mCurrentSettings = MenuItem(menu, "Current Settings", 3);
      MenuItem mFanSpeed =  MenuItem(menu, "Fan Speed", 3);


//this function builds the menu and connects the correct items together
void menuSetup()
{
  //add the file menu to the menu root
    menu.getRoot().add(mHomeScreen); 
    //setup the settings menu item
     
    mHomeScreen.addRight(mPidSettings); 
    
    mPidSettings.addRight(mPidSetPoint);
       
       mPidInputSensor.addBefore(mPidSetPoint);
       mPidInputSensor.addLeft(mPidSettings);
       
       mPidOutputChannel.addBefore(mPidInputSensor);
       mPidOutputChannel.addLeft(mPidSettings);
       
       mPidKp.addBefore(mPidOutputChannel);
       mPidKp.addLeft(mPidSettings);
       
       mPidKi.addBefore(mPidKp);
       mPidKi.addLeft(mPidSettings);
       
       mPidKd.addBefore(mPidKi);
       mPidKd.addLeft(mPidSettings);
       
       mPidDirection.addBefore(mPidKd);
       mPidDirection.addLeft(mPidSettings);
       
       mPidSampleTime.addBefore(mPidDirection);    
       mPidSampleTime.addLeft(mPidSettings);
       
       mPidAMode.addBefore(mPidSampleTime);
       mPidAMode.addLeft(mPidSettings);
       
       mPidADelta.addBefore(mPidAMode);
       mPidADelta.addLeft(mPidSettings);
       
       mPidAKp.addBefore(mPidADelta);
       mPidAKp.addLeft(mPidSettings);
       
       mPidAKi.addBefore(mPidAKp);
       mPidAKi.addLeft(mPidSettings);
       
       mPidAKd.addBefore(mPidAKi);
       mPidAKd.addLeft(mPidSettings);
           
       mPidSetPoint.addLeft(mPidSettings);
    
    
    mPidAutoTuneSettings.addBefore(mPidSettings);
    mPidAutoTuneSettings.addLeft(mHomeScreen);
    mPidAutoTuneSettings.addRight(mPidATuneInputNoise);
    
      mPidATuneOutputStep.addBefore(mPidATuneInputNoise);
      mPidATuneOutputStep.addLeft(mPidAutoTuneSettings);
      
      mPidATuneLookBack.addBefore(mPidATuneOutputStep);
      mPidATuneLookBack.addLeft(mPidAutoTuneSettings);
      
      mPidATuneControlType.addBefore(mPidATuneLookBack);
      mPidATuneControlType.addLeft(mPidAutoTuneSettings);
      
      mPidATuneEnable.addBefore(mPidATuneControlType);
      mPidATuneEnable.addLeft(mPidAutoTuneSettings);
    
      mPidATuneInputNoise.addLeft(mPidAutoTuneSettings);
    
    mCustomControlSettings.addBefore(mPidAutoTuneSettings);
    mCustomControlSettings.addLeft(mHomeScreen);
    mCustomControlSettings.addRight(mCustomControlInputSensor);
    
      mCustomControlOutputChannel.addBefore(mCustomControlInputSensor);
      mCustomControlOutputChannel.addLeft(mCustomControlSettings);
      
      
    
      mCustomControlInputSensor.addLeft(mCustomControlSettings);
    
    mSSRSettings.addBefore(mCustomControlSettings);
    mSSRSettings.addLeft(mHomeScreen);
    mSSRSettings.addRight(mSSRMinOn);
    
      mSSRMaxOff.addBefore(mSSRMinOn);
      mSSRMaxOff.addLeft(mSSRSettings);
      
      mSSRPeriod.addBefore(mSSRMaxOff);
      mSSRPeriod.addLeft(mSSRSettings);
     
      mSSRMinOn.addLeft(mSSRSettings);
      
    mRelaySettings.addBefore(mSSRSettings);
    mRelaySettings.addLeft(mHomeScreen);
    mRelaySettings.addRight(mRelayMinOn);
    
      mRelayMaxOff.addBefore(mRelayMinOn);
      mRelayMaxOff.addLeft(mRelaySettings);
      
      mRelayPeriod.addBefore(mRelayMaxOff);
      mRelayPeriod.addLeft(mRelaySettings);
     
      mRelayMinOn.addLeft(mRelaySettings);
        
    mGeneralSettings.addBefore(mRelaySettings);
    mGeneralSettings.addLeft(mHomeScreen);
    mGeneralSettings.addRight(mSaveCurrentSettings);
    
      mCurrentSettings.addBefore(mSaveCurrentSettings);
      mCurrentSettings.addLeft(mGeneralSettings);
      
      mFanSpeed.addBefore(mCurrentSettings);
      mFanSpeed.addLeft(mGeneralSettings);
      
      mSaveCurrentSettings.addLeft(mGeneralSettings);

    mPidSettings.addLeft(mHomeScreen);
    
    // Close the top level
    mPidSettings.addLeft(mHomeScreen);   
    
    mHomeScreen.addAfter(mHomeScreen);
    


}

/*
  This is an important function
  Here we get a notification whenever the user changes the menu
  That is, when the menu is navigated
*/
void menuChangeEvent(MenuChangeEvent changed){
  lcd.clear();
  lcd.setCursor(0, 0);

 if ( changed.to.isEqual(mHomeScreen)) {
    lcdPrintDash(); 
 } else {
    lcd.print(changed.to.getName());
 }
 // PID
 
 if (changed.to.isEqual(mPidSetPoint)){
   lcdPrintFloatSecondLine(pidSetPoint);
 }
 
 if (changed.to.isEqual(mPidInputSensor)){
   lcd.setCursor(0, 1);
   lcdPrintSensorAddress(configuration.pidInputSensor);
 }
 
 if (changed.to.isEqual(mPidOutputChannel)) {
    lcdPrintIntSecondLine(configuration.pidOutputChannel);
 }
 
 if (changed.to.isEqual(mPidKp)){
   lcdPrintFloatSecondLine(configuration.pidKp);
 }
 
 if (changed.to.isEqual(mPidKi)){
   lcdPrintFloatSecondLine(configuration.pidKi);
 }
 
 if (changed.to.isEqual(mPidKd)){
   lcdPrintFloatSecondLine(configuration.pidKd);
 }
 
  if (changed.to.isEqual(mPidDirection)){
   lcdPrintPidDirectionSecondLine(configuration.pidDirection);
 }
 
 if (changed.to.isEqual(mPidSampleTime)) {
    lcdPrintIntSecondLine(configuration.pidSampleTime);
 }
 
 if (changed.to.isEqual(mPidAMode)){
   lcdPrintBooleanSecondLine(configuration.pidAMode);
 }
 
 if (changed.to.isEqual(mPidADelta)){
   lcdPrintFloatSecondLine(configuration.pidADelta);
 }
 
 if (changed.to.isEqual(mPidAKp)){
   lcdPrintFloatSecondLine(configuration.pidAKp);
 }
 
 if (changed.to.isEqual(mPidAKi)){
   lcdPrintFloatSecondLine(configuration.pidAKi);
 }
 
 if (changed.to.isEqual(mPidAKd)){
   lcdPrintFloatSecondLine(configuration.pidAKd);
 }
 


 
 // PID Auto Tune
  if (changed.to.isEqual(mPidATuneInputNoise)){
    lcdPrintFloatSecondLine(configuration.pidATuneInputNoise);
  }
  if (changed.to.isEqual(mPidATuneOutputStep)){
    lcdPrintFloatSecondLine(configuration.pidATuneOutputStep);
  }   
  if (changed.to.isEqual(mPidATuneLookBack)){
    lcdPrintIntSecondLine(configuration.pidATuneLookBack);
  }   
  if (changed.to.isEqual(mPidATuneControlType)){
    lcdPrintPidATControlType(configuration.pidATuneControlType);
  }     
  if (changed.to.isEqual(mPidATuneEnable)){
    lcdPrintBooleanSecondLine(tuning);
  }
 
 
 // Custom Control
 if (changed.to.isEqual(mCustomControlInputSensor)){
   lcd.setCursor(0, 1);
   lcdPrintSensorAddress(configuration.customControlInputSensor);
 }
 
 if ( changed.to.isEqual(mCustomControlOutputChannel)) {
    lcdPrintIntSecondLine(configuration.customControlOutputChannel);
 }
 
 // SSR
 if (changed.to.isEqual(mSSRMinOn)){
   lcdPrintIntSecondLine(configuration.SSRMinOn);
 }
  if (changed.to.isEqual(mSSRMaxOff)){
   lcdPrintIntSecondLine(configuration.SSRMaxOff);
 }
  if (changed.to.isEqual(mSSRPeriod)){
   lcdPrintIntSecondLine(configuration.SSRPeriod);
 }
 
 // Relay
 if (changed.to.isEqual(mRelayMinOn)){
   lcdPrintIntSecondLine(configuration.relayMinOn);
 }
  if (changed.to.isEqual(mRelayMaxOff)){
   lcdPrintIntSecondLine(configuration.relayMaxOff);
 }
  if (changed.to.isEqual(mRelayPeriod)){
   lcdPrintIntSecondLine(configuration.relayPeriod);
 }
 
 // General Settings
 if ( changed.to.isEqual(mFanSpeed)) {
    lcdPrintIntSecondLine(configuration_general.fanSpeed);
 }
 if ( changed.to.isEqual(mCurrentSettings)) {
    lcdPrintIntSecondLine(configuration_general.currentSettings);
 }
  
}


void menuUseEvent(MenuUseEvent used) {
  // PID
  if (used.item.isEqual(mPidSetPoint)) {
    configuration.pidSetPoint=editFloat2( configuration.pidSetPoint, -50, 700, 0.01, 5);
    pidSetPoint=configuration.pidSetPoint;
  }  
  if (used.item.isEqual(mPidInputSensor)) {
    editSensor(configuration.pidInputSensor);
  }  
  
   if (used.item.isEqual(mPidOutputChannel)) {
    configuration.pidOutputChannel=editInt( configuration.pidOutputChannel, 1, 3, 1, 1, lcdPrintIntSecondLine);
 }
  
 if (used.item.isEqual(mPidKp)) {
    configuration.pidKp=editFloat2( configuration.pidKp, 0, 9999, .0001, 8);
 }  
  
 
 if (used.item.isEqual(mPidKi)) {
    configuration.pidKi=editFloat2( configuration.pidKi, 0, 9999, 0.0001, 8);
 }
 
 if (used.item.isEqual(mPidKd)) {
    configuration.pidKd=editFloat2( configuration.pidKd, 0, 9999, .0001, 8);
 }
 
 if (used.item.isEqual(mPidDirection)){
   configuration.pidDirection=editInt(configuration.pidDirection, 0, 1, 1, 1, lcdPrintPidDirectionSecondLine);
   pid.SetControllerDirection(configuration.pidDirection);
 }
 
 if (used.item.isEqual(mPidSampleTime)){
   configuration.pidSampleTime=editInt2(configuration.pidSampleTime, 100, 300000, 100, 4);
   pid.SetSampleTime(configuration.pidSampleTime);
 }
  
 if (used.item.isEqual(mPidAMode)) {
    configuration.pidAMode=editInt( configuration.pidAMode, 0, 1, 1, 1, lcdPrintBooleanSecondLine);
 }  
 
 if (used.item.isEqual(mPidADelta)) {
    configuration.pidADelta=editFloat2(configuration.pidADelta, 0, 200, 0.01, 5);
 }  

 if (used.item.isEqual(mPidAKp)) {
    configuration.pidAKp=editFloat2( configuration.pidAKp, 0, 9999, .0001, 8);
 }  
 
 if (used.item.isEqual(mPidAKi)) {
    configuration.pidAKi=editFloat2( configuration.pidAKi, 0, 9999, 0.0001, 8);
 }
 
 if (used.item.isEqual(mPidAKd)) {
    configuration.pidAKd=editFloat2( configuration.pidAKd, 0, 9999, .0001, 8);
 }
   
  

 // PID AT 
 
 if (used.item.isEqual(mPidATuneInputNoise)) {
    configuration.pidATuneInputNoise=editFloat2(configuration.pidATuneInputNoise, 0, 200, 0.01, 5);
    pidATune.SetNoiseBand(configuration.pidATuneInputNoise);
 }  

 if (used.item.isEqual(mPidATuneOutputStep)){
    configuration.pidATuneOutputStep=editInt2(configuration.pidATuneOutputStep, 0, 255, 1, 3);
    pidATune.SetOutputStep(configuration.pidATuneOutputStep);
 }
    
 if (used.item.isEqual(mPidATuneLookBack)){  
  configuration.pidATuneLookBack=editInt2(configuration.pidATuneLookBack, 1, 200, 1, 3);
  pidATune.SetLookbackSec(configuration.pidATuneLookBack);
 }

 if (used.item.isEqual(mPidATuneControlType)){
   configuration.pidATuneControlType=editInt( configuration.pidATuneControlType, 0, 1, 1, 1, lcdPrintPidATControlType);
   pidATune.SetControlType(configuration.pidATuneControlType);

 } 
 
 if (used.item.isEqual(mPidATuneEnable)) {
    tuning=editInt( 0, 0, 1, 1, 1, lcdPrintBooleanSecondLine);
 }
  
  // Custom Control Channel
  if (used.item.isEqual(mCustomControlInputSensor)) {
    editSensor(configuration.customControlInputSensor);
  }  
  
 if (used.item.isEqual(mCustomControlOutputChannel)) {
    configuration.customControlOutputChannel=editInt( configuration.customControlOutputChannel, 1, 3, 1, 1, lcdPrintIntSecondLine);
 }
 
 // SSR
 if (used.item.isEqual(mSSRMinOn)){
   configuration.SSRMinOn=editInt2(configuration.SSRMinOn, 20, 300000, 10, 5);
   
  }
  if (used.item.isEqual(mSSRMaxOff)){
   configuration.SSRMaxOff=editInt2(configuration.SSRMaxOff, 20, 300000, 10, 5);
 }
 if (used.item.isEqual(mSSRPeriod)){
    configuration.SSRPeriod=editInt2(configuration.SSRPeriod, 1000, 300000, 10, 5);
 }
 
 
  // Relay
 if (used.item.isEqual(mRelayMinOn)){
   configuration.relayMinOn=editInt2(configuration.relayMinOn, 500, 300000, 100, 4);
   
  }
  if (used.item.isEqual(mRelayMaxOff)){
   configuration.relayMaxOff=editInt2(configuration.relayMaxOff, 500, 300000, 100, 4);
 }
 if (used.item.isEqual(mRelayPeriod)){
    configuration.relayPeriod=editInt2(configuration.relayPeriod, 1000, 300000, 100, 4);
 }
 
 
  
  // General Settings
  if (used.item.isEqual(mFanSpeed)) {
    configuration_general.fanSpeed=editInt2( configuration_general.fanSpeed, 0, 255, 1, 3);
    analogWrite(fanPin, configuration_general.fanSpeed);
    eeprom_write_block((const void*)&configuration_general, (void*)0, sizeof(configuration));
  }
  if ( used.item.isEqual(mCurrentSettings)) {
    configuration_general.currentSettings=editInt( configuration_general.currentSettings, 1, maxCurrentSettings, 1, 1, lcdPrintIntSecondLine);
    eeprom_read_block((void*)&configuration, (void*)( configuration_general.currentSettings * sizeof(configuration)  ), sizeof(configuration));
    eeprom_write_block((const void*)&configuration_general, (void*)0, sizeof(configuration));
  }
  if ( used.item.isEqual(mSaveCurrentSettings)) {
    lcd.setCursor(0, 1);
    lcd.blink();
    lcd.print("Saving Settings");
    eeprom_write_block((const void*)&configuration_general, (void*)0, sizeof(configuration));
    eeprom_write_block((const void*)&configuration, (void*)(  configuration_general.currentSettings * sizeof(configuration)  ), sizeof(configuration));
    delay(2000);
    lcd.setCursor(0, 1);
    lcd.print(lcdBlankLine);
    lcd.noBlink();
    
  }

}


