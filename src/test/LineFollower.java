
package test;

import lejos.hardware.motor.Motor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.lcd.LCD;
import lejos.hardware.Button;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class LineFollower extends Thread{

    public void run() {
        MotorTest mControl = new MotorTest();
        EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S3);

        SampleProvider redModeSensorProvider = colorSensor.getRedMode();

        float[] sensorSampleArray = new float[redModeSensorProvider.sampleSize()];

        mControl.Forward();

        float proportionalGain = 600; // Kp
        float integralGain = 0.02f;  // Ki
        float derivativeGain = 700;  // Kd
        float integralAccumulator = 0;
        float previousError = 0;
        float minThreshold = 0.17f;
        float maxThreshold = 0.23f;
        int baseMotorSpeed = 200;

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

            int rightMotorSpeed = (int)(baseMotorSpeed + steeringAdjustment);
            int leftMotorSpeed = (int)(baseMotorSpeed + steeringAdjustment);

            mControl.SpeedAdjustRight(rightMotorSpeed);
            mControl.SpeedAdjustLeft(leftMotorSpeed);

            mControl.Forward();
            
            Delay.msDelay(10);
        }

        Motor.C.stop(true);
        Motor.D.stop(true);

        colorSensor.close();
    }
}
