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
        EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S3);

        SampleProvider redModeSensorProvider = colorSensor.getRedMode();

        float[] sensorSampleArray = new float[redModeSensorProvider.sampleSize()];

        int baseMotorSpeed = 200;

        Motor.C.setSpeed(baseMotorSpeed);
        Motor.D.setSpeed(baseMotorSpeed);
        Motor.C.forward();
        Motor.D.forward();

        float proportionalGain = 600; // Kp
        float integralGain = 0.02f;  // Ki
        float derivativeGain = 700;  // Kd


        float integralAccumulator = 0;
        float previousError = 0;
        
        float minThreshold = 0.17f;
        float maxThreshold = 0.23f;

        while (!Button.ESCAPE.isDown()) {
            redModeSensorProvider.fetchSample(sensorSampleArray, 0);
            float currentSensorReading = sensorSampleArray[0];

            LCD.clear();
            LCD.drawString("Intensity: " + (int)(currentSensorReading * 100), 0, 0);

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

            Motor.C.setSpeed(Math.max(0, leftMotorSpeed));
            Motor.D.setSpeed(Math.max(0, rightMotorSpeed));

            Motor.C.forward();
            Motor.D.forward();
            
            Delay.msDelay(10);
        }

        Motor.C.stop(true);
        Motor.D.stop(true);

        colorSensor.close();
    }
}