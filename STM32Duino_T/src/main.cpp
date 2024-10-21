#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <Thread.h>
#include <ThreadController.h>
#include <JC_Button.h>
#include <ezBuzzer.h>
#include <NewPing.h>
#include <HardwareSerial.h>
#define R1 PB0
#define R2 PB1
#define R3 PA12
#define R4 PA11
#define L1 PA4
#define L2 PA5
#define L3 PA6
#define L4 PA7
#define TRIGGER_PIN  PB6
#define ECHO_PIN  PB7
#define TX1_PIN PA9
#define RX1_PIN PA10
#define TX2_PIN PA2
#define RX2_PIN PA3
#define OLED_SCL PB8
#define OLED_SDA PB9
#define MAX_DISTANCE 20 // 最大测距范围 (厘米)
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
HardwareSerial Serial_1(1);
HardwareSerial Serial_3(3);
unsigned char Re_buf[11], counter = 0;
unsigned char sign = 0;
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE, /* clock=*/OLED_SCL, /* data=*/OLED_SDA);
ThreadController controller = ThreadController();
Thread display = Thread();
void Display(){
    //没写
}
Thread serial1 = Thread();
void S1(){
     
}
Thread serial2 = Thread();
void S2(){
    if (Serial.available()) {
        unsigned char temp = Serial.read();

        // 检测帧头，0x55
        if (temp == 0x55) {
            sign = 1;  // 标志位，表示找到了帧头
            counter = 0;
        }

        // 如果检测到帧头，并开始读取数据
        if (sign == 1) {
            Re_buf[counter++] = temp;
            if (counter == 11) {
                sign = 0;  // 重置标志
                counter = 0;

                // 处理数据帧
                if (Re_buf[1] == 0x51) {  // 加速度数据帧
                    int16_t ax = (Re_buf[3] << 8 | Re_buf[2]);
                    int16_t ay = (Re_buf[5] << 8 | Re_buf[4]);
                    int16_t az = (Re_buf[7] << 8 | Re_buf[6]);

                    float fAcc[3];
                    fAcc[0] = ax / 32768.0f * 16.0f;
                    fAcc[1] = ay / 32768.0f * 16.0f;
                    fAcc[2] = az / 32768.0f * 16.0f;
                }

                if (Re_buf[1] == 0x52) {  // 角速度数据帧
                    int16_t gx = (Re_buf[3] << 8 | Re_buf[2]);
                    int16_t gy = (Re_buf[5] << 8 | Re_buf[4]);
                    int16_t gz = (Re_buf[7] << 8 | Re_buf[6]);

                    float fGyro[3];
                    fGyro[0] = gx / 32768.0f * 2000.0f;
                    fGyro[1] = gy / 32768.0f * 2000.0f;
                    fGyro[2] = gz / 32768.0f * 2000.0f;
                }

                if (Re_buf[1] == 0x53) {  // 角度数据帧
                    int16_t roll = (Re_buf[3] << 8 | Re_buf[2]);
                    int16_t pitch = (Re_buf[5] << 8 | Re_buf[4]);
                    int16_t yaw = (Re_buf[7] << 8 | Re_buf[6]);

                    float fAngle[3];
                    fAngle[0] = roll / 32768.0f * 180.0f;
                    fAngle[1] = pitch / 32768.0f * 180.0f;
                    fAngle[2] = yaw / 32768.0f * 180.0f;

                    // Determine sign flag (SfAngle) based on angle value
                    int SfAngle[3];
                    for (int i = 0; i < 3; i++) {
                        if (fAngle[i] > 0) {
                            SfAngle[i] = 0;
                        } else {
                            SfAngle[i] = 1;
                        }
                    }

                    // Prepare All array
                    unsigned char All[8];
                    All[0] = 0x12;
                    All[1] = SfAngle[0];
                    All[2] = (unsigned char)fAngle[0];
                    All[3] = SfAngle[1];
                    All[4] = (unsigned char)fAngle[1];
                    All[5] = SfAngle[2];
                    All[6] = (unsigned char)fAngle[2];
                    All[7] = 0x5B;

                    // Combine into R, P, Y values
                    int R = (All[2] << 8) + All[1];
                    int P = (All[4] << 8) + All[3];
                    int Y = (All[6] << 8) + All[5];

                    // (Optional) Do something with R, P, Y
                }
            }
        }
    }
}

Thread getsr04 = Thread();
void Getsr04(){
    unsigned int uS = sonar.ping(); // 发出超声波并等待回声
   
}
void setup() {
    Serial.begin(115200);//这个是串口2(默认)
    Serial_1.begin(115200); 
    Serial_3.begin(115200); 
    display.onRun(Display);
    getsr04.onRun(Getsr04);
    serial1.onRun(S1);
    serial2.onRun(S2);
    display.setInterval(50);
    getsr04.setInterval(50);
    serial1.setInterval(50);
    serial2.setInterval(50);
    controller.add(&display);
    controller.add(&serial1);
    controller.add(&getsr04);
    controller.add(&serial2);
}

void loop() {
    controller.run();
}
