package test;

import lejos.hardware.motor.Motor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.lcd.LCD;
import lejos.hardware.Button;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class LineFollower {

    public static void main(String[] args) {
        // Create and configure the color sensor
        EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S3);

        // You can try & use other modes to see the difference & feel free to use the mode that
        // suites your needs the best, for instance, getAmbientMode(), getRedMode(), etc
        // Here we are setting the sensor to red mode (measures reflected red light)
        SampleProvider light = colorSensor.getRedMode();  // Use red mode for reflected light intensity

        // Create an array to hold the sensor data
        float[] sample = new float[light.sampleSize()];

        // Set motor speeds
        Motor.C.setSpeed(300);
        Motor.D.setSpeed(300);

        // Start motors moving forward
        Motor.C.forward();
        Motor.D.forward();

        // Threshold for detecting the black line
        // NOTE: You'll most probably have to fine tune the threshold value
        float minThreshold = 0.18f;  // Min value for line detection
        float maxThreshold = 0.22f;  // Max value for line detection

        // Continuously follow the line until a button is pressed
        while (!Button.ESCAPE.isDown()) {
            // Get the current red light intensity reading from the sensor
            light.fetchSample(sample, 0);  // 0 is the index where data will be stored
            
            // Display light intensity on LCD
            LCD.clear();
            LCD.drawString("Intensity: " + (int)(sample[0] * 100) + "%", 0, 0);
            
            // Determine if we're on a black line based on the threshold
            boolean isHalfLine = sample[0] >= minThreshold && sample[0] <= maxThreshold;
            
            // Display whether we're on a black line on the second line of LCD
            if (isHalfLine) {
                LCD.drawString("HALF LINE", 0, 1);
            } else {
                LCD.drawString("NOT HALF LINE", 0, 1);
            }

            // // If the light intensity is low (black line), the robot is on the line
            if (isHalfLine) { 
                Motor.C.setSpeed(300);
                Motor.D.setSpeed(300);
                Motor.C.forward();
                Motor.D.forward();
            }
            else {
                if (sample[0] > maxThreshold) {
                    Motor.C.setSpeed(300);
                    Motor.D.setSpeed(200);
                    Motor.C.forward();
                    Motor.D.forward();
                }
                else {
                    Motor.C.setSpeed(200);
                    Motor.D.setSpeed(300);
                    Motor.C.forward();
                    Motor.D.forward();
                }
            }

            // Add a small delay to reduce the frequency of updates
            Delay.msDelay(30);
        }

        // Stop the motors before exiting
        Motor.C.stop();
        Motor.D.stop();
        
        // Remember to close the sensor before exiting
        colorSensor.close();
    }
}