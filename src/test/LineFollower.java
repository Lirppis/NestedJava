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
        // init color sensor
        EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S3);

        SampleProvider redModeSensorProvider = colorSensor.getRedMode();

        float[] sensorSampleArray = new float[redModeSensorProvider.sampleSize()];

        int baseMotorSpeed = 200;

        Motor.C.setSpeed(baseMotorSpeed);
        Motor.D.setSpeed(baseMotorSpeed);
        Motor.C.forward();
        Motor.D.forward();

        // PID constants    
        float proportionalGain = 600; // Kp: current error reaction
        float integralGain = 0.02f;  // Ki: history of errors
        float derivativeGain = 700;  // Kd: future error reaction

        // history of errors
        float integralError = 0;
        // previous value of red light sensor
        float previousError = 0;
        
        // min and max threshold for smooth moves
        float minThreshold = 0.17f;
        float maxThreshold = 0.23f;

        while (!Button.ESCAPE.isDown()) {
            // get red light value 
            redModeSensorProvider.fetchSample(sensorSampleArray, 0);
            float currentSensorReading = sensorSampleArray[0];

            LCD.clear();
            LCD.drawString("Intensity: " + (int)(currentSensorReading * 100), 0, 0);

            float currentError = 0;
            float steeringAdjustment;

            // while line is between min and max threshold it doesnt change speed and direction
            if (currentSensorReading >= minThreshold && currentSensorReading <= maxThreshold) {
                steeringAdjustment = 0;
                integralError = 0;
            } else {
                // if red light value is too small it turns left
                if (currentSensorReading < minThreshold) {
                    currentError = currentSensorReading - minThreshold;
                } else {
                    // if red light value is too big it turns right
                    currentError = currentSensorReading - maxThreshold;
                }

                // history of errors
                integralError += currentError;
                // future error reaction
                float derivativeOfError = currentError - previousError;

                // PID formula
                steeringAdjustment = (proportionalGain * currentError) +
                                       (integralGain * integralError) +
                                       (derivativeGain * derivativeOfError);
            }

            previousError = currentError;

            int leftMotorSpeed = (int)(baseMotorSpeed + steeringAdjustment);
            int rightMotorSpeed = (int)(baseMotorSpeed - steeringAdjustment);

            // Math.max is for preventing negative speed
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