#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Byte.h>

#define motorPin1 11 // IN1
#define motorPin2 10 // IN2
#define motorPin3 9 // IN3
#define motorPin4 8 // IN4
 
#define step 256//4096 // 1바퀴 스텝수

#define melodyPin 15
 
#define NOTE_E7  2637
#define NOTE_C7  2093
#define NOTE_G7  3136
#define NOTE_G6  1568
#define NOTE_A6  1760
#define NOTE_B6  1976
#define NOTE_AS6 1865
#define NOTE_E6  1319
#define NOTE_A7  3520
#define NOTE_F7  2794
#define NOTE_C6  1047
#define NOTE_B7  3951
#define NOTE_AS7 3729
#define NOTE_GS7 3322
#define NOTE_D7  2349

int tempo[] = {
  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,
 
  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,
 
  9, 9, 9,
  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,
 
  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,
 
  9, 9, 9,
  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,
};
int melody[] = {
  NOTE_E7, NOTE_E7, 0, NOTE_E7,
  0, NOTE_C7, NOTE_E7, 0,
  NOTE_G7, 0, 0,  0,
  NOTE_G6, 0, 0, 0,
 
  NOTE_C7, 0, 0, NOTE_G6,
  0, 0, NOTE_E6, 0,
  0, NOTE_A6, 0, NOTE_B6,
  0, NOTE_AS6, NOTE_A6, 0,
 
  NOTE_G6, NOTE_E7, NOTE_G7,
  NOTE_A7, 0, NOTE_F7, NOTE_G7,
  0, NOTE_E7, 0, NOTE_C7,
  NOTE_D7, NOTE_B6, 0, 0,
 
  NOTE_C7, 0, 0, NOTE_G6,
  0, 0, NOTE_E6, 0,
  0, NOTE_A6, 0, NOTE_B6,
  0, NOTE_AS6, NOTE_A6, 0,
 
  NOTE_G6, NOTE_E7, NOTE_G7,
  NOTE_A7, 0, NOTE_F7, NOTE_G7,
  0, NOTE_E7, 0, NOTE_C7,
  NOTE_D7, NOTE_B6, 0, 0
};

int pinArray[4] = { motorPin1, motorPin2, motorPin3, motorPin4 };
int CW[8] = {
  0b1000,
  0b1100,
  0b0100,
  0b0110,
  0b0010,
  0b0011,
  0b0001,
  0b1001
};
 
int CCW[8] = {
  0b1000,
  0b1001,
  0b0001,
  0b0011,
  0b0010,
  0b0110,
  0b0100,
  0b1100
};

const int trigPin = 2;   // 초음파 센서의 Trig 핀
const int echoPin = 3;   // 초음파 센서의 Echo 핀
const int stopDistance = 10;  // 일정 거리 (cm)

int a=0;
ros::NodeHandle  nh;

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);
char hello[] = "Mission Complete!";

void sing(int s) {
  
    int noteDuration = 1000;
    int pauseBetweenNotes = noteDuration * 1.30;
    Serial.println(" 'Mario Theme'");
    int size = sizeof(melody) / sizeof(int);
    for (int thisNote = 0; thisNote < size; thisNote++) {
 
      // to calculate the note duration, take one second
      // divided by the note type.
      //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
      int noteDuration = 1000 / tempo[thisNote];
 
      buzz(melodyPin, melody[thisNote], noteDuration);
 
      // to distinguish the notes, set a minimum time between them.
      // the note's duration + 30% seems to work well:
      int pauseBetweenNotes = noteDuration * 1.30;
      delay(pauseBetweenNotes);
 
      // stop the tone playing:
      buzz(melodyPin, 0, noteDuration);
 
    }
  }

void buzz(int targetPin, long frequency, long length) {
  digitalWrite(13, HIGH);
  long delayValue = 1000000 / frequency / 2; // calculate the delay value between transitions
  //// 1 second's worth of microseconds, divided by the frequency, then split in half since
  //// there are two phases to each cycle
  long numCycles = frequency * length / 1000; // calculate the number of cycles for proper timing
  //// multiply frequency, which is really cycles per second, by the number of seconds to
  //// get the total number of cycles to produce
  for (long i = 0; i < numCycles; i++) { // for the calculated length of time...
    digitalWrite(targetPin, HIGH); // write the buzzer pin high to push out the diaphram
    delayMicroseconds(delayValue); // wait for the calculated delay value
    digitalWrite(targetPin, LOW); // write the buzzer pin low to pull back the diaphram
    delayMicroseconds(delayValue); // wait again or the calculated delay value
  }
 
}

void messageCb( const std_msgs::Byte& led_msg) {

  if (led_msg.data == 1){
    a=1;
  }
  else if (led_msg.data == 2){
    a=2;
  }
  else if (led_msg.data == 0){
    a=0;
  }
  
    }


ros::Subscriber<std_msgs::Byte> sub("led_out", messageCb );


void setup() {
  for (int i = 0; i < 4; i++) pinMode(pinArray[i], OUTPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(melodyPin, OUTPUT);

  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(chatter);
}

void loop() {
  nh.spinOnce();
  // 초음파 센서로 거리 측정
  float distance = measureDistance();
  if (a == 1)
    {
      for (int i = 0; i < step; i++) {
        int temp = i % 8;
        digitalWrite(pinArray[0], bitRead(CW[temp], 0));
        digitalWrite(pinArray[1], bitRead(CW[temp], 1));
        digitalWrite(pinArray[2], bitRead(CW[temp], 2));
        digitalWrite(pinArray[3], bitRead(CW[temp], 3));
        delay(0.01);

    }}
    else if( a == 2)
    {
      for (int i = 0; i < step; i++) {
        int temp = i % 8;
        digitalWrite(pinArray[0], bitRead(CCW[temp], 0));
        digitalWrite(pinArray[1], bitRead(CCW[temp], 1));
        digitalWrite(pinArray[2], bitRead(CCW[temp], 2));
        digitalWrite(pinArray[3], bitRead(CCW[temp], 3));
        delay(0.01);
        if (distance <= stopDistance) {
          a=0;
          str_msg.data = hello;
          chatter.publish( &str_msg );
          sing(1);
          return;
    }
  }}
    else if( a == 0)
    {
      stopMotor();
    }
  }

// 거리 측정 함수
float measureDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  float duration = pulseIn(echoPin, HIGH);
  float distance = duration * 0.034 / 2;  // 음파의 속력은 초당 34cm

  return distance;
}


void stopMotor() {
  for (int i = 0; i < 4; i++) digitalWrite(pinArray[i], LOW);
}