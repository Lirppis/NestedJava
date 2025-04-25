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

    public static final EV3UltrasonicSensor ultrasonicSensor;
    public static final EV3ColorSensor colorSensor;
    public static final SampleProvider distanceProvider;
    public static final SampleProvider colorProvider;
    public static final float[] distanceSample;
    public static final float[] colorSample;

    public static final float DANGER_DISTANCE = 0.20f;

    static {
        ultrasonicSensor = new EV3UltrasonicSensor(SensorPort.S4);
        colorSensor = new EV3ColorSensor(SensorPort.S3);

        distanceProvider = ultrasonicSensor.getDistanceMode();
        colorProvider = colorSensor.getRedMode();

        distanceSample = new float[distanceProvider.sampleSize()];
        colorSample = new float[colorProvider.sampleSize()];
    }

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
        LCD.drawString("Starting...", 0, 0);
    
        LineFollower lineFollowerThread = new LineFollower();
        ObjectAvoid objectAvoidThread = new ObjectAvoid();

        LCD.drawString("Starting threads", 0, 1);
        lineFollowerThread.start();
        objectAvoidThread.start();

        LCD.drawString("Press ESCAPE exit", 0, 4);
        Button.ESCAPE.waitForPressAndRelease();

        LCD.drawString("Stopping...", 0, 5);

        Delay.msDelay(500);

        Motor.D.stop(true);
        Motor.C.stop(true); 
        Main.closeSensors();
    }
}
    
    


