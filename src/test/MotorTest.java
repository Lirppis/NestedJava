package test;

import lejos.hardware.motor.Motor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.lcd.LCD;
import lejos.hardware.Button;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class MotorTest extends Thread{
    int baseMotorSpeed = 200;
    int leftMotorSpeed;
    int rightMotorSpeed;
    
    public void Forward(){
    Motor.C.setSpeed(baseMotorSpeed);
    Motor.D.setSpeed(baseMotorSpeed);
    Motor.C.forward();
    Motor.D.forward();
    }

    public void SpeedAdjustRight(int rightMotorSpeed){
        Motor.C.setSpeed(Math.max(0, rightMotorSpeed));
    }

    public void SpeedAdjustLeft(int leftMotorSpeed){
        Motor.D.setSpeed(Math.max(0, leftMotorSpeed));
    }

    public void StopMotors(){
        Motor.D.stop(true);
        Motor.C.stop(true);
    }

}
