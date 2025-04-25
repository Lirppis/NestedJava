package test;

import lejos.hardware.lcd.LCD;
import lejos.hardware.Button;
import lejos.utility.Delay;
import lejos.hardware.motor.Motor;

public class LineFollower extends Thread {

    // PID constants
    private final float proportionalGain = 600; // Kp
    private final float integralGain = 0.02f;  // Ki
    private final float derivativeGain = 700;  // Kd
    private float integralError = 0; // sum of errors
    private float previousError = 0; 
    
    private final float minThreshold = 0.17f; // min accepted value of sensor
    private final float maxThreshold = 0.23f; // max accepted value of sensor
    private final int baseMotorSpeed = 200;

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
