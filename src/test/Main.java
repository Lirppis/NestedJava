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

/**
 * Main class for the LEGO EV3 robot application.
 * <p>
 * Initializes sensors, launches the LineFollower and ObjectAvoid threads,
 * and handles shutdown on user command.
 * Provides static utility methods for sensor readings and cleanup.
 * </p>
 */
public class Main {
    /**
     * Shared flag indicating whether object avoidance is active.
     */
    public static final AtomicBoolean isAvoiding = new AtomicBoolean(false);

    /** Ultrasonic sensor for measuring distance to obstacles. */
    static EV3UltrasonicSensor ultrasonicSensor;
    /** Color sensor for line detection in red mode. */
    static EV3ColorSensor colorSensor;
    /** SampleProvider for distance measurements. */
    static SampleProvider distanceProvider;
    /** SampleProvider for color (red) measurements. */
    static SampleProvider colorProvider;
    /** Buffer array for distance samples. */
    static float[] distanceSample;
    /** Buffer array for color samples. */
    static float[] colorSample;

    /**
     * Distance threshold in meters at which the robot begins avoidance behavior.
     */
    public static final float DANGER_DISTANCE = 0.20f;

    /**
     * Closes both sensors to free underlying hardware resources.
     */
    public static void closeSensors() {
        ultrasonicSensor.close();
        colorSensor.close();
    }

    /**
     * Reads the latest distance measurement from the ultrasonic sensor.
     * 
     * @return the current distance to an object in meters
     */
    public static float getDistance() {
        distanceProvider.fetchSample(distanceSample, 0);
        return distanceSample[0];
    }

    /**
     * Reads the latest red intensity from the color sensor.
     * 
     * @return the normalized red light value (0.0–1.0)
     */
    public static float getColorReading() {
        colorProvider.fetchSample(colorSample, 0);
        return colorSample[0];
    }

    /**
     * Entry point of the program.
     * <p>
     * Initializes sensors, instantiates and starts the LineFollower and ObjectAvoid threads,
     * and waits for the ESCAPE button press to exit and clean up.
     * </p>
     * 
     * @param args command-line arguments (not used)
     */
    public static void main(String[] args) {
        try {
            // initialize sensors
            ultrasonicSensor = new EV3UltrasonicSensor(SensorPort.S4);
            colorSensor = new EV3ColorSensor(SensorPort.S3);

            // configure sample providers and buffers
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

        // Initialize behavior threads
        LineFollower lineFollowerThread = new LineFollower();
        ObjectAvoid objectAvoidThread = new ObjectAvoid();

        LCD.drawString("Starting threads", 0, 1);
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
