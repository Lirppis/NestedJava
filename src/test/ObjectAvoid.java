package test;

import lejos.hardware.lcd.LCD;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;
import lejos.hardware.port.SensorPort;
import lejos.hardware.motor.Motor;
import lejos.utility.Delay;
import lejos.hardware.Button;

public class ObjectAvoid {

    static EV3UltrasonicSensor ultrasonicSensor = new EV3UltrasonicSensor(SensorPort.S4);
    static SampleProvider distance = ultrasonicSensor.getDistanceMode();
    static float[] sample = new float[distance.sampleSize()];
    static float dangerDist = 0.20f;
    static int baseSpeed = 200;

     private static void stopMotors() {
        Motor.C.stop(true);
        Motor.D.stop(true);
        Delay.msDelay(100); 
    }

    public static void main(String[] args) {
        System.out.println("Simple Object Avoidance Started.");
        LCD.clear();
        LCD.drawString("Program START", 0, 0);

        while (!Button.ESCAPE.isDown()) {
             distance.fetchSample(sample, 0);
             float currentDistance = sample[0];

             LCD.drawString("Dist: " + String.format("%.2f", currentDistance) + "m  ", 0, 5);

             if (currentDistance < dangerDist && currentDistance > 0) {
                 LCD.drawString("Avoiding...      ", 0, 4);
                 stopMotors();
                 Motor.C.setSpeed(baseSpeed);
                 Motor.D.setSpeed(baseSpeed);
                 Motor.C.forward();
                 Motor.D.backward();

                 float turnCheckDistance;
                 float safetyMargin = 0.10f;
                 do {
                     Delay.msDelay(20);
                     distance.fetchSample(sample, 0);
                     turnCheckDistance = sample[0];
                 } while (turnCheckDistance < (dangerDist + safetyMargin) && turnCheckDistance >= 0 && !Button.ESCAPE.isDown());

                 stopMotors();
                 LCD.clear(4);

             } else {
                 LCD.drawString("Moving Forward   ", 0, 4);
                 if (!Motor.C.isMoving() || !Motor.D.isMoving()) {
                      Motor.C.setSpeed(baseSpeed);
                      Motor.D.setSpeed(baseSpeed);
                      Motor.C.forward();
                      Motor.D.forward();
                 }
             }

            //with the speed of 200 the robot should go forward approx. 30cm in 3.5 seconds 
            try {
                Thread.sleep(3500);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            // stop the motors
            Motor.C.stop(true);
            Motor.D.stop(true);

            //turn the robot slightly to left 
            // if no more obstacle, should follow the line
            // if obstacle, do the while 
            Motor.C.rotate(-120, true);
            Motor.D.rotate(120, true);

             Delay.msDelay(50);
         }

         stopMotors();
         ultrasonicSensor.close();
         LCD.clear();
         System.out.println("Program exiting.");
     }
 }
