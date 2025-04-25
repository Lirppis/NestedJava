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
    private float integralAccumulator = 0;
    private float previousError = 0;
    private final float minThreshold = 0.17f;
    private final float maxThreshold = 0.23f;
    private final int baseMotorSpeed = 200;

    @Override
    public void run() {
        LCD.drawString("LineFollower running", 0, 2);
        while (!Button.ESCAPE.isDown()) {

            if (Main.isAvoiding.get()) {
                LCD.drawString("LF Avoiding", 0, 7);
                Delay.msDelay(50);
                continue;
            }

            float currentDistance = Main.getDistance();
            LCD.drawString("LF Dist: " + String.format("%.2f", currentDistance), 0, 1);

            if (currentDistance > 0 && currentDistance < Main.DANGER_DISTANCE) {
                LCD.drawString("Obstacle detected!", 0, 6);
                Motor.D.stop(true);
                Motor.C.stop(true);
                LCD.drawString("LF Motors Stopped", 0, 7);
                Delay.msDelay(500);
                integralAccumulator = 0;
                previousError = 0;
                Main.isAvoiding.set(true);
                continue;
            }
            float currentSensorReading = Main.getColorReading();
            LCD.clear(0);
            LCD.drawString("Intensity: " + (int)(currentSensorReading * 100), 0, 0);
            LCD.drawString("Following Line   ", 0, 6);

            float currentError = 0;
            float steeringAdjustment;

            if (currentSensorReading >= minThreshold && currentSensorReading <= maxThreshold) {
                steeringAdjustment = 0;
                integralAccumulator = 0;
            } else {
                if (currentSensorReading < minThreshold) {
                    currentError = currentSensorReading - minThreshold;
                } else {
                    currentError = currentSensorReading - maxThreshold;
                }
                integralAccumulator += currentError;
                float derivativeOfError = currentError - previousError;
                steeringAdjustment = (proportionalGain * currentError) +
                                       (integralGain * integralAccumulator) +
                                       (derivativeGain * derivativeOfError);
            }
            previousError = currentError;

            int leftMotorSpeed = (int)(baseMotorSpeed + steeringAdjustment);
            int rightMotorSpeed = (int)(baseMotorSpeed - steeringAdjustment);

            LCD.drawString("LF Set L:" + leftMotorSpeed + " R:" + rightMotorSpeed, 0, 7);

            Motor.D.setSpeed(Math.max(0, leftMotorSpeed));  // Left Motor (Port D)
            Motor.C.setSpeed(Math.max(0, rightMotorSpeed)); // Right Motor (Port C)
            Motor.D.forward();
            Motor.C.forward();

            Delay.msDelay(10);
        }

        Motor.D.stop(true);
        Motor.C.stop(true);
        LCD.drawString("LineFollower stopped", 0, 2);
    }
}
