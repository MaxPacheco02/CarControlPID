/*!
 * @file Car_Control.cpp
 * @brief Car Control - Analysis of Control Systems
 * @n Header file for Car Control
 *
 * @author [Demian](A01368643@tec.mx)
 * @author [Max](A01552369@tec.mx)
 * @version  V1.0.0
 * @date  2023-10-14
 */

#include "Car_Control.h"

Car_Control::Car_Control(){
    pid = PID_Control(sample_time, Kp, Ki, Kd, u_max, u_min);
}

int Car_Control::updateDirection(){ //Reads from sensor
    sensIzq = digitalRead(sensorIzqPin); 
    sensDer = digitalRead(sensorDerPin); 
    sensCent = digitalRead(sensorCentralPin);
    sens = (sensIzq << 2) | (sensCent << 1) | sensDer;
    switch(sens){
        case 1:
            dir = 2;
            break;
        case 2:
            dir = 0;
            break;
        case 3:
            dir = 1;
            break;
        case 4:
            dir = -2;
            break;
        case 6:
            dir = -1;
            break;
        case 7:
            dir = 0;
            break;
    }
    return dir;
}

int Car_Control::updateServo(int newDir){
    int servo_return = 0;
    servo_val = map(newDir, -2, 2, 50, 110);
    if(servo_last != servo_val){
        servo_return = servo_val;
    }
    servo_last = servo_val;
    return servo_return;
}

void Car_Control::updateMotorSpeed(){ 
    if(Serial.available()){
        setpoint = Serial.parseFloat();
        pid.updateReferences(setpoint);

    }
    pid.saturateManipulation(speed);
    pwm = pid.u_;
    analogWrite(ENA, pwm);


    // Serial.println(pid.u_);
    // int n = 0;
    // if(!rand){
    //     if(Serial.available())
    //         n = Serial.parseInt();
        
    //     if(n == 1){
    //         rand = true;
    //         pwm_r = 100;
    //     }
    // } else {
    //     if(pwm != pwm_r){
    //         pwm = pwm_r;
    //         analogWrite(ENA,pwm);
    //     }
    // }

        // currentTimeMot = millis();
        // if ((currentTimeMot - lastUpdateTimeMot) >= dtRand) {
        //     lastUpdateTimeMot = currentTimeMot;
        //     if(pwm == 0){
        //         pwm = 100;            
        //         dtRand = 500;

        //     }
        //     else{
        //         pwm = 0;
        //         dtRand = random(50,1000);

        //     }
        //     analogWrite(ENA,pwm);
        // }
}

void Car_Control::readSpeed(){
    currentTime = millis();
    if ((currentTime - lastUpdateTime) >= dt) {
        noInterrupts(); // Disable interrupts to safely read encoderCount
        motor_speed = (encoderCount - lastEncoderCount) / dt; // Calculate speed (counts per second)
        interrupts(); // Re-enable interrupts
        speed = motor_speed * rads_per_rev * radio / pulse_per_rev * 100; // [dm/s]
        lastEncoderCount = encoderCount;
        lastUpdateTime = currentTime;
    }

    currentTimePrint = millis();
    if((currentTimePrint - lastUpdateTimePrint) >= print_dt){
        buf[0] = (currentTime/10000)%10 + '0';
        buf[1] = (currentTime/1000)%10 + '0';
        buf[2] = '.';
        buf[3] = (currentTime/100)%10 + '0';
        buf[4] = (currentTime/10)%10 + '0';
        buf[5] = (int)(speed) + '0';
        buf[6] = '.';
        buf[7] = ((int)(speed*10.0))%10 + '0';
        buf[8] = ((int)(speed*100.0))%10 + '0';
        buf[9] = (pwm/100) + '0';
        buf[10] = (pwm/10)%10 + '0';
        buf[11] = (pwm%10) + '0';
        buf[12] = '/';
        Serial.write(buf,13);
        Serial.println();
        lastUpdateTimePrint = currentTimePrint;
    }
}