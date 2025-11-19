/*!
 * @file DFRobot_C4001.cpp
 * @brief Define the basic structure of the DFRobot_C4001 class, the implementation of the basic methods
 * @copyright	Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license The MIT License (MIT)
 * @author [ZhixinLiu](zhixin.liu@dfrobot.com)
 * @version V1.0
 * @date 2024-02-02
 * @url https://github.com/DFRobot/DFRobot_C4001
 */
#include "DFRobot_C4001.h"

DFRobot_C4001::DFRobot_C4001(){}
DFRobot_C4001::~DFRobot_C4001(){}

sSensorStatus_t DFRobot_C4001::getStatus(void)
{
  sSensorStatus_t data;
  uint8_t temp[100] = {0};
  uint8_t len = 0;
  sAllData_t allData;
  readReg(0, temp, 100);
  writeReg(0, (uint8_t *)START_SENSOR, strlen(START_SENSOR));
  while(len == 0){
    sleepForMillis(1000);
    len = readReg(0, temp, 100);
    allData = anaysisData(temp ,len);
  }
  data.workStatus = allData.sta.workStatus;
  data.workMode   = allData.sta.workMode;
  data.initStatus = allData.sta.initStatus;
  return data;
}

bool DFRobot_C4001::motionDetection(void)
{
  static bool old = false;
  uint8_t status = 0;
  uint8_t len = 0;
  uint8_t temp[100] = {0};
  sAllData_t data;
  len = readReg(0, temp, 100);
  data = anaysisData(temp ,len);
  if(data.exist){
    old = (bool)status;
    return (bool)data.exist;
  }else{
    return (bool)old;
  }
}

void DFRobot_C4001::setSensor(eSetMode_t mode)
{
  uint8_t temp = mode;
  if(mode == eStartSen){
    writeReg(0, (uint8_t *)START_SENSOR, strlen(START_SENSOR));
    sleepForMillis(200);  // must timer
  }else if(mode == eStopSen){
    writeReg(0, (uint8_t *)STOP_SENSOR, strlen(STOP_SENSOR));
    sleepForMillis(200);  // must timer
  }else if(mode == eResetSen){
    writeReg(0, (uint8_t *)RESET_SENSOR, strlen(RESET_SENSOR));
    sleepForMillis(1500);  // must timer
  }else if(mode == eSaveParams){
    writeReg(0, (uint8_t *)STOP_SENSOR, strlen(STOP_SENSOR));
    sleepForMillis(200);  // must timer
    writeReg(0, (uint8_t *)SAVE_CONFIG, strlen(SAVE_CONFIG));
    sleepForMillis(800);  // must timer
    writeReg(0, (uint8_t *)START_SENSOR, strlen(START_SENSOR));
  }else if(mode == eRecoverSen){
    writeReg(0, (uint8_t *)STOP_SENSOR, strlen(STOP_SENSOR));
    sleepForMillis(200);
    writeReg(0, (uint8_t *)RECOVER_SENSOR, strlen(RECOVER_SENSOR));
    sleepForMillis(800);  // must timer
    writeReg(0, (uint8_t *)START_SENSOR, strlen(START_SENSOR));
    sleepForMillis(500);
  }
}

bool DFRobot_C4001::setSensorMode(eMode_t mode)
{
  sensorStop();
  if(mode == eExitMode){
    writeReg(0, (uint8_t *)EXIST_MODE, strlen(EXIST_MODE));
    sleepForMillis(50);  
  }else{
    writeReg(0, (uint8_t *)SPEED_MODE, strlen(SPEED_MODE));
    sleepForMillis(50);
  }
  sleepForMillis(50);
  writeReg(0, (uint8_t *)SAVE_CONFIG, strlen(SAVE_CONFIG));
  sleepForMillis(500);
  writeReg(0, (uint8_t *)START_SENSOR, strlen(START_SENSOR));
  sleepForMillis(100);
  return true;
}

bool DFRobot_C4001::setTrigSensitivity(uint8_t sensitivity)
{
  uint8_t temp = sensitivity;
  if(sensitivity > 9){
    return false;
  }
  string data = "setSensitivity 255 1";
  data[19] = sensitivity + 0x30;
  writeCMD(data, data, (uint8_t)1);
  return true;
}

uint8_t DFRobot_C4001::getTrigSensitivity(void)
{
  sResponseData_t responseData;
  uint8_t temp[100] = {0};
  readReg(0, temp, 100);
  string data = "getSensitivity";
  responseData = wRCMD(data, (uint8_t)1);
  if(responseData.status){ 
    return responseData.response1;
  }
  return 0;
}

bool DFRobot_C4001::setKeepSensitivity(uint8_t sensitivity)
{
  uint8_t temp = sensitivity;
  if(sensitivity > 9){
    return false;
  }
  string data = "setSensitivity 1 255";
  data[15] = sensitivity + 0x30;
  writeCMD(data, data, (uint8_t)1);
  return true;
}

uint8_t DFRobot_C4001::getKeepSensitivity(void)
{
  sResponseData_t responseData;
  uint8_t temp[100] = {0};
  readReg(0, temp, 100);
  string data = "getSensitivity";
  responseData = wRCMD(data, (uint8_t)1);
  if(responseData.status){
    return responseData.response2;
  }
  return 0;
}

bool DFRobot_C4001::setDelay(uint8_t trig , uint16_t keep)
{
  if(trig > 200){
    return false;
  }
  if(keep < 4 || keep > 3000){
    return false;
  }
  string data = "setLatency ";
  data += string((float)trig*0.01, 1);
  data += " ";
  data += string((float)keep*0.5, 1);
  writeCMD(data, data, (uint8_t)1);
  return true;
}

uint8_t DFRobot_C4001::getTrigDelay(void)
{
  sResponseData_t responseData;
  string data = "getLatency";
  responseData = wRCMD(data, (uint8_t)1);
  if(responseData.status){
    return responseData.response1*100;
  }
  return 0;
}

uint16_t DFRobot_C4001::getKeepTimerout(void)
{
  sResponseData_t responseData;
  string data = "getLatency";
  responseData = wRCMD(data, (uint8_t)2);
  if(responseData.status){
    return responseData.response2*2;
  }
  return 0;
}

bool DFRobot_C4001::setDetectionRange(uint16_t min, uint16_t max, uint16_t trig)
{
  if(max < 240 || max > 2000){
    return false;
  }
  if(min < 30 || min > max){
    return false;
  }
  string data1 = "setRange ";
  string data2 = "setTrigRange ";
  data1 += string(((float)min)/100.0, 1);
  data1 += " ";
  data1 += string(((float)max)/100.0, 1);
  data2 += string(((float)trig)/100.0, 1);
  writeCMD(data1, data2, (uint8_t)2);
  return true;
}

uint16_t DFRobot_C4001::getTrigRange(void)
{
  sResponseData_t responseData;
  string data = "getTrigRange";
  responseData = wRCMD(data, (uint8_t)1);
  if(responseData.status){
    return responseData.response1*100;
  }
  return 0;
}

uint16_t DFRobot_C4001::getMaxRange(void)
{
  sResponseData_t responseData;
  string data = "getRange";
  responseData = wRCMD(data, (uint8_t)2);
  if(responseData.status){
    return responseData.response2*100;
  }
  return 0;
}

uint16_t DFRobot_C4001::getMinRange(void)
{
  sResponseData_t responseData;
  string data = "getRange";
  responseData = wRCMD(data, (uint8_t)2);
  if(responseData.status){
    return responseData.response1*100;
  }
  return 0;
}

uint8_t DFRobot_C4001::getTargetNumber(void)
{
  static uint8_t flash_number = 0;
  uint8_t temp[10] = {0};
  uint8_t len = 0;
  uint8_t temp[100] = {0};
  sAllData_t data;
  len = readReg(0, temp, 100);
  data = anaysisData(temp ,len);
  if(data.target.number != 0){
    flash_number = 0;
    _buffer.number = data.target.number;
    _buffer.range  = data.target.range/100.0;
    _buffer.speed  = data.target.speed/100.0;
    _buffer.energy = data.target.energy;
  }else{
    _buffer.number = 1;
    if(flash_number++ > 10){
      _buffer.number = 0;
      _buffer.range  = 0;
      _buffer.speed  = 0;
      _buffer.energy = 0;
    }
  }
  return data.target.number;
}

float DFRobot_C4001::getTargetSpeed(void)
{
  return _buffer.speed;
}

float DFRobot_C4001::getTargetRange(void)
{
  return _buffer.range;
}

uint32_t DFRobot_C4001::getTargetEnergy(void)
{
  return _buffer.energy;
}

bool DFRobot_C4001::setDetectThres(uint16_t min, uint16_t max, uint16_t thres)
{
  if(max > 2500){
    return false;
  }
  if(min > max){
    return false;
  }
  string data1 = "setRange ";
  string data2 = "setThrFactor ";
  data1 += string(((float)min)/100.0, 1);
  data1 += " ";
  data1 += string(((float)max)/100.0, 1);
  data2 += thres;
  writeCMD(data1, data2, (uint8_t)2);
  return true;
}

bool DFRobot_C4001::setIoPolaity(uint8_t value)
{
  if(value > 1){
    return false;
  }
  string data = "setGpioMode 1 ";
  data += value;
  writeCMD(data, data, (uint8_t)1);
  return true;
}

uint8_t DFRobot_C4001::getIoPolaity(void)
{
  sResponseData_t responseData;
  string data = "getGpioMode 1";
  responseData = wRCMD(data, (uint8_t)2);
  if(responseData.status){
    return responseData.response2;
  }
  return 0;
}

bool DFRobot_C4001::setPwm(uint8_t pwm1 , uint8_t pwm2, uint8_t timer)
{
  if(pwm1 > 100 || pwm2 > 100){
    return false;
  }
  string data = "setPwm ";
  data += pwm1;
  data += " ";
  data += pwm2;
  data += " ";
  data += timer;
  writeCMD(data, data, (uint8_t)1);
  return true;
}

sPwmData_t DFRobot_C4001::getPwm(void)
{
  sPwmData_t pwmData;
  memset(&pwmData, 0, sizeof(sPwmData_t));
  sResponseData_t responseData;
  string data = "getPwm";
  responseData = wRCMD(data, (uint8_t)3);
  if(responseData.status){
    pwmData.pwm1 = responseData.response1;
    pwmData.pwm2 = responseData.response2;
    pwmData.timer = responseData.response3;
  }
  return pwmData;
}

uint16_t DFRobot_C4001::getTMinRange(void)
{
  sResponseData_t responseData;
  string data = "getRange";
  responseData = wRCMD(data, (uint8_t)1);
  if(responseData.status){
    return responseData.response1*100;
  }
  return 0;
}

uint16_t DFRobot_C4001::getTMaxRange(void)
{
  sResponseData_t responseData;
  string data = "getRange";
  responseData = wRCMD(data, (uint8_t)2);
  if(responseData.status){
    return responseData.response2*100;
  }
  return 0;
}

uint16_t DFRobot_C4001::getThresRange(void)
{
  sResponseData_t responseData;
  string data = "getThrFactor";
  responseData = wRCMD(data, (uint8_t)1);
  if(responseData.status){
    return responseData.response1;
  }
  return 0;
}

void DFRobot_C4001::setFrettingDetection(eSwitch_t sta)
{
   string data = "setMicroMotion ";
  data += sta;
  writeCMD(data, data, (uint8_t)1);
}

eSwitch_t DFRobot_C4001::getFrettingDetection(void)
{
  sResponseData_t responseData;
  string data = "getMicroMotion";
  responseData = wRCMD(data, (uint8_t)1);
  if(responseData.status){
    return (eSwitch_t)responseData.response1;
  }
  return (eSwitch_t)0;
}

sResponseData_t DFRobot_C4001::anaysisResponse(uint8_t *data, uint8_t len ,uint8_t count)
{
  sResponseData_t responseData;
  uint8_t space[5] = {0};
  uint8_t i = 0;
  uint8_t j = 0;
  for(i = 0; i < len; i++){
    if(data[i] == 'R' && data[i+1] == 'e' && data[i+2] == 's'){
      break;
    }
  }
  if(i == len || i == 0){
    responseData.status = false;
  }else{
    responseData.status = true;
    for(j = 0; i < len; i++){
      if(data[i] == ' '){
        space[j++] = i + 1;
      }
    }
    if(j != 0){
      responseData.response1 = atof((const char*)(data+space[0]));
      if(j >= 2){
        responseData.response2 = atof((const char*)(data+space[1]));
      }
      if(count == 3){
        responseData.response3 = atof((const char*)(data+space[2]));
      }
    }else{
      responseData.response1 = 0.0;
      responseData.response2 = 0.0;
    }
  }
  return responseData;
}

sAllData_t DFRobot_C4001::anaysisData(uint8_t * data, uint8_t len) //data is an array and len is length of data probably
{
  sAllData_t allData;
  uint8_t location = 0;
  memset(&allData, 0, sizeof(sAllData_t)); //Sets memory to 000... for size of sAllData_t struct

  for(uint8_t i = 0; i < len; i++){ //Iterates through data array to find '$'
    if(data[i] == '$'){
      location = i;
      break;
    }
  }

  if(location == len){
    return allData;
  }

  if(0 == strncmp((const char *)(data+location), "$DFHPD", strlen("$DFHPD"))){ //Checks if $DFGHPD is in data //HPD = Human presence detection
    allData.sta.workMode = eExitMode; //Exit mode vs speed mode. Speed mode detects object velocity, exit mode detects if there is object
    allData.sta.workStatus = 1;
    allData.sta.initStatus = 1;
    if(data[location+7] == '1'){ //If return 1 at location + 7 then there is an object
      allData.exist = 1;
    }else{
      allData.exist = 0;
    }
  }else if(0 == strncmp((const char *)(data+location), "$DFDMD", strlen("$DFDMD"))){ //Checks if $DFDDMD is in data
    // $DFDMD,par1,par2,par3,par4,par5,par6,par7*
    allData.sta.workMode = eSpeedMode; 
    allData.sta.workStatus = 1;
    allData.sta.initStatus = 1;
    char *token;
    char *parts[10]; // Let's say there are at most 10 parts //???
    int index = 0;   // Used to track the number of parts stored
    token = strtok((char*)(data+location), ","); //splits data into pointers to tokens split from data using ,
    while (token != NULL) {
      parts[index] = token; // Stores partial Pointers in an array
      if(index++ > 8){
        break;
      }
      token = strtok(NULL, ","); // Continue to extract the next section 
    }
    allData.target.number = atoi(parts[1]);
    allData.target.range = atof(parts[3]) * 100;
    allData.target.speed = atof(parts[4]) * 100;
    allData.target.energy = atof(parts[5]);
  }else{
  }
  return allData;
}

sResponseData_t DFRobot_C4001::wRCMD(string cmd1, uint8_t count)
{
  uint8_t len = 0;
  uint8_t temp[200] = {0};
  sResponseData_t responseData;
  sensorStop();
  writeReg(0, (uint8_t *)cmd1.c_str(), cmd1.length());
  sleepForMillis(100);
  len = readReg(0, temp, 200);
  responseData = anaysisResponse(temp, len, count);
  sleepForMillis(100);
  writeReg(0, (uint8_t *)START_SENSOR, strlen(START_SENSOR));
  sleepForMillis(100);
  return responseData;
}

void DFRobot_C4001::writeCMD(string cmd1 , string cmd2, uint8_t count)
{
  sensorStop();
  writeReg(0, (uint8_t *)cmd1.c_str(), cmd1.length());
  sleepForMillis(100);
  if(count > 1){
    sleepForMillis(100);
    writeReg(0, (uint8_t *)cmd2.c_str(), cmd2.length());
    sleepForMillis(100);
  }
  writeReg(0, (uint8_t *)SAVE_CONFIG, strlen(SAVE_CONFIG));
  sleepForMillis(100);
  writeReg(0, (uint8_t *)START_SENSOR, strlen(START_SENSOR));
  sleepForMillis(100);
}

bool DFRobot_C4001::sensorStop(void)
{
  uint8_t len = 0;
  uint8_t temp[200] = {0};
  writeReg(0, (uint8_t *)STOP_SENSOR, strlen(STOP_SENSOR));
  sleepForMillis(1000);
  len = readReg(0, temp, 200);
  while(1){
    if(len != 0){
      if (strstr((const char *)temp, "sensorStop") != NULL) {
        return true;
      }
    }
    memset(temp, 0, 200);
    sleepForMillis(400);
    writeReg(0, (uint8_t *)STOP_SENSOR, strlen(STOP_SENSOR));
    len = readReg(0, temp, 200);
    
  }
}

DFRobot_C4001_UART::DFRobot_C4001_UART(int port, uint32_t Baud ,uint8_t txpin, uint8_t rxpin)
{
  this->serial_port = port;
  this->_baud = Baud;
  this->_txpin = txpin;
  this->_rxpin = rxpin;
}

bool DFRobot_C4001_UART::begin()
{ 
  struct termios tty;
  
  // Read in existing settings, and handle any error
  if(tcgetattr(serial_port, &tty) != 0) {
      printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
      return 1;
  }

  //Control modes
  tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
  tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
  tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size
  tty.c_cflag |= CS8; // 8 bits per byte (most common)
  tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
  tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

  //Local modes
  tty.c_lflag &= ~ICANON;
  tty.c_lflag &= ~ECHO; // Disable echo
  tty.c_lflag &= ~ECHOE; // Disable erasure
  tty.c_lflag &= ~ECHONL; // Disable new-line echo
  tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP

  //Input modes
  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
  tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

  //Output modes
  tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
  tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
  // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
  // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

  //Special characters
  tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
  tty.c_cc[VMIN] = 0;

  cfsetispeed(&tty, this->_baud);
  cfsetospeed(&tty, this->_baud);
  
  // Save tty settings, also checking for error
  if (tcsetattr(this->serial_port, TCSANOW, &tty) != 0) {
      printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
      return 1;
  }
  return true;
}

void DFRobot_C4001_UART::writeReg(uint8_t reg, uint8_t *data, uint8_t len)
{
  write(serial_port, data, len);
  len = reg;
}

int16_t DFRobot_C4001_UART::readReg(uint8_t reg, uint8_t *data, uint8_t len)
{
  uint16_t i = 0;
  auto nowtime = std::chrono::system_clock::now();
  while(static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::system_clock::now() - nowtime).count()) < TIME_OUT){
    while(_serial->available() > 0){
      if(i == len) return len;
      unsigned char read_buf;
      read(this->serial_port, &read_buf, 1);
      data[i++] = (int)read_buf;
    }
  }
  len = reg;
  reg = len;
  return i;
}

void sleepForMillis(int time){
  std::this_thread::sleep_for(std::chrono::milliseconds(time));
}