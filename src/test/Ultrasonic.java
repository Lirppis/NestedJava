package test;

import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.lcd.LCD;
import lejos.hardware.Button;
import lejos.robotics.SampleProvider;   // allows the sensor to return the samples or data
                                        // e.g., for getting distance data from sonic sensor etc

public class Ultrasonic {

    public static void main(String[] args) {
        // Creating an instance of US sensor at port 4
        EV3UltrasonicSensor ultrasonicSensor = new EV3UltrasonicSensor(SensorPort.S4);
        
        // Get the distance sample provider
        SampleProvider distance = ultrasonicSensor.getDistanceMode();
        
        // Create a sample array to hold the distance value
        // even though sonic sensor gives distance as an o/p, but since other sensors, e.g., light sensor
        // can provide multiple values, therefore to keep consistency, I'm using sampleprovider
        float[] sample = new float[distance.sampleSize()];
        float maxdistance = 0.1f;
        // Keep displaying the distance, until user presses a button
        while (!Button.ESCAPE.isDown())
        {
            // Get the curRent distnce reading from the US sensor
            distance.fetchSample(sample, 0);
            
            // Display the distance on the LCD screen
            LCD.clear();
            LCD.drawString("Dist: " + sample[0] + " meters", 0, 0);
            if (sample[0] > maxdistance && sample[0] < 3.0f){
                maxdistance = sample[0];
                LCD.drawString("Highest:" + maxdistance, 0, 1);
            } else {
                LCD.drawString("Highest:" + maxdistance, 0, 1);
            }
            // Refresh display every 100 ms
            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

        }
        
        // Close US sensor
        ultrasonicSensor.close();
    }
}