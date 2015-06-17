


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
      MenuItem mFanSpeed =  MenuItem(menu, "Fan Speed", 3);
      MenuItem mSaveCurrentSettings = MenuItem(menu, "Save Settings", 3);
      MenuItem mCurrentSettings = MenuItem(menu, "Current Settings", 3);


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
       
       mPidAMode.addBefore(mPidDirection);
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
    
    mCustomControlSettings.addBefore(mPidSettings);
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
    mGeneralSettings.addRight(mFanSpeed);
    
      mSaveCurrentSettings.addBefore(mFanSpeed);
      mSaveCurrentSettings.addLeft(mGeneralSettings);
      
      mCurrentSettings.addBefore(mSaveCurrentSettings);
      mCurrentSettings.addLeft(mGeneralSettings);
      
      mFanSpeed.addLeft(mGeneralSettings);

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
 
 
 if (changed.to.isEqual(mPidInputSensor)){
   lcd.setCursor(0, 1);
   lcdPrintSensorAddress(configuration.pidInputSensor);
 }
 
  if ( changed.to.isEqual(mPidOutputChannel)) {
    lcdPrintIntSecondLine(configuration.pidOutputChannel);
 }
 
 // Custom Control
 
 if (changed.to.isEqual(mCustomControlInputSensor)){
   lcd.setCursor(0, 1);
   lcdPrintSensorAddress(configuration.customControlInputSensor);
 }
 
 if ( changed.to.isEqual(mCustomControlOutputChannel)) {
    lcdPrintIntSecondLine(configuration.customControlOutputChannel);
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
    configuration.pidSetPoint=editFloat( configuration.pidSetPoint, -50, 700, 0.1, 4, lcdPrintFloatSecondLine);
    pidSetPoint=configuration.pidSetPoint;
  }  
  if (used.item.isEqual(mPidInputSensor)) {
    editSensor(configuration.pidInputSensor);
  }  
  
   if (used.item.isEqual(mPidOutputChannel)) {
    configuration.pidOutputChannel=editInt( configuration.pidOutputChannel, 1, 3, 1, 1, lcdPrintIntSecondLine);
 }
  
 if (used.item.isEqual(mPidKp)) {
    configuration.pidKp=editFloat( configuration.pidKp, 0, 2000, 1, 3, lcdPrintFloatSecondLine);
 }  
  
 
 if (used.item.isEqual(mPidKi)) {
    configuration.pidKi=editFloat( configuration.pidKi, 0, 10, 0.01, 3, lcdPrintFloatSecondLine);
 }
 
 if (used.item.isEqual(mPidKd)) {
    configuration.pidKd=editFloat( configuration.pidKd, 0, 10, .01, 3, lcdPrintFloatSecondLine);
 }
  
  
  // Custom Control Channel
  if (used.item.isEqual(mCustomControlInputSensor)) {
    editSensor(configuration.customControlInputSensor);
  }  
  
 if (used.item.isEqual(mCustomControlOutputChannel)) {
    configuration.customControlOutputChannel=editInt( configuration.customControlOutputChannel, 1, 3, 1, 1, lcdPrintIntSecondLine);
 }
  
  // General Settings
  if (used.item.isEqual(mFanSpeed)) {
    configuration_general.fanSpeed=editInt( configuration_general.fanSpeed, 0, 255, 1, 3, lcdPrintIntSecondLine);
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


