#include "DFRobot_C4001.h"

int main()
{
  /* -----------------------------
   *  Open UART device
   *  Change if needed:
   *    /dev/ttyUSB0
   *    /dev/ttyUSB1
   *    /dev/ttyS0
   * ----------------------------- */
  int serial_port = open("/dev/ttyTODO", O_RDWR);
  
  if (serial_port < 0) {
    perror("Failed to open serial port");
    return -1;
  }

  DFRobot_C4001_UART radar(serial_port, B9600);

  while (!radar.begin()) {
    std::cout << "NO Devices !" << std::endl;
    sleepForMillis(1000);
  }

  std::cout << "Device connected!" << std::endl;

  // speed Mode
  radar.setSensorMode(eSpeedMode);

  sSensorStatus_t data = radar.getStatus();

  //  0 stop  1 start
  std::cout << "work status  = " << (int)data.workStatus << std::endl;
  //  0 is exist   1 speed
  std::cout << "work mode    = " << (int)data.workMode << std::endl;
  //  0 no init    1 init success
  std::cout << "init status  = " << (int)data.initStatus << std::endl;
  std::cout << std::endl;

  /*
   * min Detection range Minimum distance, unit cm, range 0.3~20m (30~2000), not exceeding max, otherwise the function is abnormal.
   * max Detection range Maximum distance, unit cm, range 2.4~20m (240~2000)
   * thres Target detection threshold, dimensionless unit 0.1, range 0~6553.5 (0~65535)
   */
  if (radar.setDetectThres(/*min*/11, /*max*/1200, /*thres*/10)) {
    std::cout << "Set detect threshold successfully" << std::endl;
  }

  // set Fretting Detection
  radar.setFrettingDetection(eON);

  // get confige params
  std::cout << "min range = " << radar.getTMinRange() << std::endl;
  std::cout << "max range = " << radar.getTMaxRange() << std::endl;
  std::cout << "threshold range = " << radar.getThresRange() << std::endl;
  std::cout << "fretting detection = " << radar.getFrettingDetection() << std::endl;

  std::cout << std::endl;

  /* -----------------------------
   *  Main loop
   * ----------------------------- */
  while (true) {
    std::cout << "target number = " << (int)radar.getTargetNumber() << std::endl;

    std::cout << "target speed  = " << radar.getTargetSpeed() << " m/s" << std::endl;

    std::cout << "target range  = " << radar.getTargetRange() << " m" << std::endl;

    std::cout << "target energy = " << radar.getTargetEnergy() << std::endl;

    std::cout << "-----------------------------" << std::endl;

    sleepForMillis(100);
  }

  close(serial_port);
  return 0;
}
