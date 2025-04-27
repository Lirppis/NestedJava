package test;

import lejos.hardware.lcd.LCD;
import lejos.hardware.Button;
import lejos.utility.Delay;
import lejos.hardware.motor.Motor;

/**
 * LineFollower thread continuously follows a line using PID control,
 * while coordinating with the Main thread for obstacle avoidance.
 * <p>
 * It reads sensor values, computes steering adjustments, and drives motors D and C.
 * It also stops and triggers avoidance when obstacles are detected.
 * </p>
 */
public class LineFollower extends Thread {

    /** Proportional gain (Kp) for PID control. */
    private final float proportionalGain = 600; // Kp
    /** Integral gain (Ki) for PID control. */
    private final float integralGain = 0.02f;  // Ki
    /** Derivative gain (Kd) for PID control. */
    private final float derivativeGain = 700;  // Kd
    /** Accumulated integral error for PID control. */
    private float integralError = 0;
    /** Previous cycle error, used for derivative calculation. */
    private float previousError = 0;
    /** Minimum threshold sensor reading to consider after calibration. */
    private final float minThreshold = 0.17f;
    /** Maximum threshold sensor reading to consider after calibration. */
    private final float maxThreshold = 0.23f;
    /** Base speed for both motors before PID adjustment. */
    private final int baseMotorSpeed = 200;

    /**
     * Main execution loop of the LineFollower thread.
     * <p>
     * This method repeatedly:
     * <ul>
     *   <li>Displays status on the LCD.</li>
     *   <li>Checks for obstacle avoidance signals from Main.</li>
     *   <li>Reads obstacle distance and triggers avoidance if too close.</li>
     *   <li>Reads line sensor values and computes PID steering adjustment.</li>
     *   <li>Applies motor speed updates and moves the motors.</li>
     * </ul>
     * The loop runs until the ESCAPE button is pressed.
     * </p>
     * @see Main#isAvoiding
     * @see Main#getDistance()
     * @see Main#getColorReading()
     */
    @Override
    public void run() {
        LCD.drawString("LineFollower running", 0, 2);
        while (!Button.ESCAPE.isDown()) {

            // check signal from main thread about object avoidance
            if (Main.isAvoiding.get()) {
                LCD.drawString("LF Avoiding", 0, 7);
                Delay.msDelay(50);
                continue;
            }

            // current distance to object if exists
            float currentDistance = Main.getDistance();
            LCD.drawString("LF Dist: " + String.format("%.2f", currentDistance), 0, 1);

            // if object detected and in danger distance you signal to main thread to start avoidance
            if (currentDistance > 0 && currentDistance < Main.DANGER_DISTANCE) {
                LCD.drawString("Obstacle detected!", 0, 6);
                Motor.D.stop(true);
                Motor.C.stop(true);

                Delay.msDelay(500);

                integralError = 0;
                previousError = 0;

                Main.isAvoiding.set(true);
                continue;
            }
            // current red light sensor value
            float currentSensorReading = Main.getColorReading();

            LCD.drawString("Following Line   ", 0, 6);

            float currentError = 0;
            float steeringAdjustment;

            // if sensor value is between min and max threshold do nothing for smoother movement
            if (currentSensorReading >= minThreshold && currentSensorReading <= maxThreshold) {
                steeringAdjustment = 0;
                integralError = 0;
            } else {
                // if sensor value is below min threshold, turn left
                if (currentSensorReading < minThreshold) {
                    currentError = currentSensorReading - minThreshold;
                } else {
                    // if sensor value is above max threshold, turn right
                    currentError = currentSensorReading - maxThreshold;
                }
                // calculate integral of errors
                integralError += currentError;
                float derivativeOfError = currentError - previousError;
                // PID formula
                steeringAdjustment = (proportionalGain * currentError) +
                                   (integralGain * integralError) +
                                   (derivativeGain * derivativeOfError);
            }
            previousError = currentError;

            // calculate motor speed
            int leftMotorSpeed = (int)(baseMotorSpeed + steeringAdjustment);
            int rightMotorSpeed = (int)(baseMotorSpeed - steeringAdjustment);

            Motor.D.setSpeed(Math.max(0, leftMotorSpeed));
            Motor.C.setSpeed(Math.max(0, rightMotorSpeed));
            Motor.D.forward();
            Motor.C.forward();

            Delay.msDelay(10);
        }

        Motor.D.stop(true);
        Motor.C.stop(true);
        LCD.drawString("LineFollower stopped", 0, 2);
    }
}
