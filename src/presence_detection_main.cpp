#include "DFRobot_C4001.h"

int main()
{
    int serial_port = open("/dev/ttyTODO", O_RDWR);

    if (serial_port < 0) {
        perror("Failed to open serial port");
        return -1;
    }

    DFRobot_C4001_UART radar(serial_port, B9600);

    while (!radar.begin()) {
        std::cout << "NO Devices !" << std::endl;
        radar.sleepForMillis(1000);
    }

    std::cout << "Device connected!" << std::endl;

    // exist Mode
    radar.setSensorMode(eExitMode);

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
    * trig Detection range Maximum distance, unit cm, default trig = max
    */
    if (radar.setDetectionRange(30, 1000, 1000))
        std::cout << "Set detection range successfully!" << std::endl;

    // set trigger sensitivity 0 - 9
    if (radar.setTrigSensitivity(1))
        std::cout << "Set trig sensitivity successfully!" << std::endl;

    // set keep sensitivity 0 - 9
    if (radar.setKeepSensitivity(2))
        std::cout << "Set keep sensitivity successfully!" << std::endl;

    /*
    * trig Trigger delay, unit 0.01s, range 0~2s (0~200)
    * keep Maintain the detection timeout, unit 0.5s, range 2~1500 seconds (4~3000)
    */
    if (radar.setDelay(100, 4))
        std::cout << "Set delay successfully!" << std::endl;
    /*
    * pwm1  When no target is detected, the duty cycle of the output signal of the OUT pin ranges from 0 to 100
    * pwm2  After the target is detected, the duty cycle of the output signal of the OUT pin ranges from 0 to 100
    * timer The value ranges from 0 to 255, corresponding to timer x 64ms
    *        For example, timer=20, it takes 20*64ms=1.28s for the duty cycle to change from pwm1 to pwm2.
    */
    if (radar.setPwm(50, 0, 10))
        std::cout << "Set PWM period successfully!" << std::endl;
    /*
    * Serial module valid
    * Set pwm polarity
    * 0：Output low level when there is a target, output high level when there is no target
    * 1: Output high level when there is a target, output low level when there is no target (default)
    */
    if (radar.setIoPolaity(1))
        std::cout << "Set IO polarity successfully!" << std::endl;

    // get confige params
    std::cout << "trig sensitivity = " << (int)radar.getTrigSensitivity() << std::endl;
    std::cout << "keep sensitivity = " << (int)radar.getKeepSensitivity() << std::endl;
    std::cout << "min range = " << radar.getMinRange() << std::endl;
    std::cout << "max range = " << radar.getMaxRange() << std::endl;
    std::cout << "trig range = " << radar.getTrigRange() << std::endl;
    std::cout << "keep time = " << radar.getKeepTimerout() << std::endl;
    std::cout << "trig delay = " << (int)radar.getTrigDelay() << std::endl;
    std::cout << "polarity = " << (int)radar.getIoPolaity() << std::endl;

    sPwmData_t pwmData = radar.getPwm();
    std::cout << "pwm1 = " << (int)pwmData.pwm1 << std::endl;
    std::cout << "pwm2 = " << (int)pwmData.pwm2 << std::endl;
    std::cout << "pwm timer = " << (int)pwmData.timer << std::endl;

    while (true) {
        if (radar.motionDetection()) {
            std::cout << "exist motion" << std::endl << std::endl;
        }
        radar.sleepForMillis(100);
    }

    close(serial_port);
    return 0;
}
