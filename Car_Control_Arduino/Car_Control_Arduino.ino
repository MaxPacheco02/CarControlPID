#include <Car_Control.h>
#include <Servo.h>

Servo MG995_Servo; // Instance of Servo

int serv = 0;
Car_Control car;
int limit_low = 0;
int limit_high = 6;

void setup() {
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);  
  pinMode(ENA,OUTPUT);

  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderPinA), updateEncoder, CHANGE);

  digitalWrite(IN1,LOW); // rotate forward
  digitalWrite(IN2,HIGH);

  pinMode(sensorIzqPin, INPUT) ;
  pinMode(sensorDerPin, INPUT) ;
  pinMode(sensorCentralPin, INPUT) ;
  
  MG995_Servo.attach(Servo_PWM); // Connect D6 of Arduino with pwm signal pin of servo motor
//  analogWrite(ENA,90);
  Serial.begin(115200);
}

void loop() {  
//  Serial.print(car.pwm);
//  Serial.print(" ");

//  Serial.print(car.speed);
//  Serial.print(",");
//  Serial.print(limit_low);
//  Serial.print(",");
//  Serial.println(limit_high);

  
//  Serial.print(" ");
//  Serial.println(car.pid.u_);
  car.readSpeed();
  car.updateMotorSpeed();

  serv = car.updateServo(car.updateDirection());
  if(serv != 0)
    MG995_Servo.write(serv);
}

void updateEncoder() {// Called when encoder changes state
  if (digitalRead(encoderPinB) == digitalRead(encoderPinA))
    car.encoderCount++;
  else
    car.encoderCount--;
}
