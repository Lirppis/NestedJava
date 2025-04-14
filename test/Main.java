package test;

import lejos.robotics.RegulatedMotor; // Port D: Left motor | Port C: Right motor
import lejos.hardware.motor.Motor;
import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;
import lejos.utility.Delay;
import lejos.hardware.sensor.EV3UltrasonicSensor; // Port 4
import lejos.robotics.SampleProvider;
import lejos.hardware.port.SensorPort;

public class Main extends Thread{
    EV3UltrasonicSensor ultrasonicSensor = new EV3UltrasonicSensor(SensorPort.S4);
    SampleProvider distance = ultrasonicSensor.getDistanceMode();
    float[] sample = new float[distance.sampleSize()];
    float dangerDist = 0.3f;
    
    static void stopMotors() {
        Motor.D.startSynchronization();
        // Stopping both motors
        Motor.D.flt();
        Motor.C.flt();
        Motor.D.endSynchronization();
    }

    static void startMotors() {
        //Syncing the motors and moving forward
        Motor.D.startSynchronization();
        Motor.D.forward();                          
        Motor.C.forward();   
        // Ending the sync, needs to be done everytime stopping or changing motor behaviour                       
        Motor.D.endSynchronization();
    }

    public static void main(String[] args) {
        RegulatedMotor[] syncList = {Motor.C};
        // Syncing motor D with the list
        Motor.D.synchronizeWith(syncList);

        Thread sonic = new Main();
        //Thread motor = new MotorThread();
        // Creating a list of motors that can be synchronized with  
        

            
        // Printing the code version to be sure that a new version was uploaded         
        LCD.drawString("Test version 4.12", 0, 1);
        startMotors(); 
        sonic.start();  
        Delay.msDelay(3000);
        stopMotors();                            
        LCD.drawString("Reached halfway.", 0, 2);
        LCD.drawString("Returning to base.", 0, 3);
        

        //Rotating both motors 410 degrees to opposite directions
        Motor.D.startSynchronization();
        Motor.D.rotate(410);
        Motor.C.rotate(-410);
        Motor.D.endSynchronization();
        // Waiting for the rotation to finnish 
        while (Motor.D.isMoving()){
            Thread.yield();
        }

        // Return to starting position
        startMotors();
        sonic.start();
        Delay.msDelay(2000);

        stopMotors();
        Button.waitForAnyPress();
        }

        public void run(){
            while (Motor.D.isMoving() && ultrasonicSensor.isEnabled()) {
                LCD.drawString("Thread running", 0, 6);
                distance.fetchSample(sample, 0);
                if (sample[0] < dangerDist){
                    stopMotors();
                    LCD.drawString("Obstacle detected at", 0, 4);
                    LCD.drawString("" + sample[0], 0, 5);
                    break;
                    }
                }
            }
        }
    
    


