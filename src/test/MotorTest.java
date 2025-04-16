package test;

import lejos.robotics.RegulatedMotor; // Port D: Left motor | Port C: Right motor
import lejos.hardware.motor.Motor;
import lejos.hardware.motor.NXTRegulatedMotor;
import lejos.hardware.lcd.LCD;
import lejos.utility.Delay;
import lejos.hardware.sensor.EV3UltrasonicSensor; // Port 4
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;

public class MotorTest extends Thread{
    EV3UltrasonicSensor ultrasonicSensor = new EV3UltrasonicSensor(SensorPort.S4);
    // Create a list to ensure that the motors are running in sync
    RegulatedMotor[] syncList = {Motor.C};
    SonicSensor sonicS = new SonicSensor();
    
    // Stopping motors
    public void stopMotors() {
        //Syncing the motors and moving forward
        Motor.D.synchronizeWith(syncList);
        Motor.D.startSynchronization();
        // Stopping both motors without braking
        Motor.D.flt();
        Motor.C.flt();
        Motor.D.endSynchronization();
        // Ending the sync, needs to be done everytime stopping or changing motor behaviour  
        ultrasonicSensor.disable();
    }

    // Strating motors
    public void startMotors() {
        ultrasonicSensor.enable();
        Motor.D.synchronizeWith(syncList);
        Motor.D.startSynchronization();
        Motor.D.forward();                          
        Motor.C.forward();                        
        Motor.D.endSynchronization();
    }

    // Turning
    public void turnMotors(){
        Motor.D.synchronizeWith(syncList);
        //Rotating both motors 410 degrees to opposite directions
        Motor.D.startSynchronization();
        Motor.D.rotate(410);
        Motor.C.rotate(-410);
        Motor.D.endSynchronization();
        // Waiting for the rotation to finnish 
        while (Motor.D.isMoving()){
            Thread.yield();
        }
    }

}



