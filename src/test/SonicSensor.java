package test;

import lejos.hardware.lcd.LCD;
import lejos.hardware.sensor.EV3UltrasonicSensor; // Port 4
import lejos.robotics.SampleProvider;
import lejos.hardware.port.SensorPort;

public class SonicSensor extends Thread{

    EV3UltrasonicSensor ultrasonicSensor = new EV3UltrasonicSensor(SensorPort.S4);
    SampleProvider distance = ultrasonicSensor.getDistanceMode();
    float[] sample = new float[distance.sampleSize()];
    // The distance which the ultrasonic sensor will stop at
    float dangerDist = 0.3f;
    Main main = new Main();
    MotorTest motorTest = new MotorTest();

    public void run(){
        while (ultrasonicSensor.isEnabled()) {
            distance.fetchSample(sample, 0);
            LCD.drawString("Thread running ", 0, 4);
            if (sample[0] < dangerDist){
                motorTest.stopMotors();
                LCD.drawString("Obstacle detected " + sample[0], 0, 5);
                LCD.drawString("" + sample[0], 0, 6);
            }
        }
    }

}



// Hacky way of running the loop for x amount of time
        // Checking if there is an obstacle infront of the robot, with ultrasopnic sensor
        /*for (timeI = 0; timeI < 20; timeI++){
            distance.fetchSample(sample, 0);
            if (sample[0] < dangerDist){
                stopMotors();
                LCD.drawString("Obstacle detected " + sample[0], 0, 4);
                break;
            }
            Delay.msDelay(100);
        } */