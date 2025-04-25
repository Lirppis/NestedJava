package test;

import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;
import lejos.utility.Delay;
import lejos.hardware.motor.Motor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.port.SensorPort;
import lejos.robotics.SampleProvider;
import java.util.concurrent.atomic.AtomicBoolean;

public class Main {
    public static final AtomicBoolean isAvoiding = new AtomicBoolean(false);

    static EV3UltrasonicSensor ultrasonicSensor;
    static EV3ColorSensor colorSensor;
    static SampleProvider distanceProvider;
    static SampleProvider colorProvider;
    static float[] distanceSample;
    static float[] colorSample;

    // max distance to object before start avoidance
    public static final float DANGER_DISTANCE = 0.20f;


    public static void closeSensors() {
        ultrasonicSensor.close();
        colorSensor.close();
    }

    public static float getDistance() {
        distanceProvider.fetchSample(distanceSample, 0);
        return distanceSample[0];
    }

    public static float getColorReading() {
        colorProvider.fetchSample(colorSample, 0);
        return colorSample[0];
    }


    public static void main(String[] args) {    
        try {
            // initialize sensors   
            ultrasonicSensor = new EV3UltrasonicSensor(SensorPort.S4);
            colorSensor = new EV3ColorSensor(SensorPort.S3);

            // get distance and color samples
            distanceProvider = ultrasonicSensor.getDistanceMode();
            colorProvider = colorSensor.getRedMode();
            distanceSample = new float[distanceProvider.sampleSize()];
            colorSample = new float[colorProvider.sampleSize()];

        } catch (Exception e) {
            LCD.clear();
            LCD.drawString("Sensor Init Failed:", 0, 0);
            Delay.msDelay(5000);
            return;
        }
        // linefollower init
        LineFollower lineFollowerThread = new LineFollower();
        // object avoid init
        ObjectAvoid objectAvoidThread = new ObjectAvoid();

        LCD.drawString("Starting threads", 0, 1);
        // start threads
        lineFollowerThread.start();
        objectAvoidThread.start();

        LCD.drawString("Press ESC to exit", 0, 4);
        Button.ESCAPE.waitForPressAndRelease();
        
        Delay.msDelay(500);

        Motor.D.stop(true);
        Motor.C.stop(true); 
        Main.closeSensors();
    }
}
    
    


