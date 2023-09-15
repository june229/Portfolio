#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <Servo.h>

Servo servoMotor1;
Servo servoMotor2;
const int servoPin1 = 5;  // 1번 서보모터의 핀 번호
const int servoPin2 = 6;  // 2번 서보모터의 핀 번호

int a = 0;
ros::NodeHandle  nh;

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);
char hello[] = "Mission Complete!";

void messageCb(const std_msgs::Int32& led_msg) {
  if (led_msg.data == 1) {
    a = 1;
  } else if (led_msg.data == 2) {
    a = 2;
  } else if (led_msg.data == 0) {
    a = 0;
  }
}

ros::Subscriber<std_msgs::Int32> sub("led_out", messageCb);

void setup() {
  servoMotor1.attach(servoPin1);
  servoMotor2.attach(servoPin2);

  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(chatter);
}

void loop() {
  nh.spinOnce();

  if (a == 1) {
    servoMotor1.write(90);  // 1번 서보모터를 중앙 위치로 회전
    delay(2000);            // 2초 동안 대기
    servoMotor2.write(80);  // 2번 서보모터를 중앙 위치로 회전
  } else if (a == 2) {
    servoMotor1.write(90);  // 1번 서보모터를 중앙 위치로 회전
  } else if (a == 0) {
    servoMotor1.write(25);  // 1번 서보모터를 초기 위치로 회전
    servoMotor2.write(0);  // 2번 서보모터를 초기 위치로 회전
  }
}
