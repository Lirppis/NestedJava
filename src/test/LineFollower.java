package test;

import lejos.hardware.motor.Motor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.lcd.LCD;
import lejos.hardware.Button;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class LineFollower  {

    public static void main(String[] args) {
        EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S3);

        SampleProvider redModeSensorProvider = colorSensor.getRedMode();

        float[] sensorSampleArray = new float[redModeSensorProvider.sampleSize()];

        int baseMotorSpeed = 250;

        Motor.C.setSpeed(baseMotorSpeed);
        Motor.D.setSpeed(baseMotorSpeed);
        Motor.C.forward();
        Motor.D.forward();

        float minThreshold = 0.18f;
        float maxThreshold = 0.22f;

        float targetSensorValue = (minThreshold + maxThreshold) / 2.0f;

        float proportionalGain = 500; // Kp: Adjusts reaction strength to current error
        float integralGain = 0.01f;  // Ki: Adjusts reaction strength to accumulated error
        float derivativeGain = 500;  // Kd: Adjusts reaction strength to rate of change of error


        float integralAccumulator = 0;
        float previousError = 0;

        while (!Button.ESCAPE.isDown()) {
            redModeSensorProvider.fetchSample(sensorSampleArray, 0);
            float currentSensorReading = sensorSampleArray[0];

            LCD.clear();
            LCD.drawString("Intensity: " + (int)(currentSensorReading * 100), 0, 0);
            LCD.drawString("Target:    " + (int)(targetSensorValue * 100), 0, 1);

            float currentError = currentSensorReading - targetSensorValue;

            integralAccumulator += currentError;

            float derivativeOfError = currentError - previousError;

            float steeringAdjustment = (proportionalGain * currentError) +
                                       (integralGain * integralAccumulator) +
                                       (derivativeGain * derivativeOfError);

            int leftMotorSpeed = (int)(baseMotorSpeed + steeringAdjustment);
            int rightMotorSpeed = (int)(baseMotorSpeed - steeringAdjustment);

            Motor.C.setSpeed(Math.max(0, leftMotorSpeed));
            Motor.D.setSpeed(Math.max(0, rightMotorSpeed));

            Motor.C.forward();
            Motor.D.forward();
            
            previousError = currentError;

            Delay.msDelay(10);
        }

        Motor.C.stop(true);
        Motor.D.stop(true);

        colorSensor.close();
    }
}