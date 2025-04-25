
package test;

import lejos.hardware.lcd.LCD;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;
import lejos.hardware.port.SensorPort;
import lejos.hardware.motor.Motor;
import lejos.utility.Delay;
import lejos.hardware.Button;

public class ObjectAvoid extends Thread{

    static EV3UltrasonicSensor ultrasonicSensor = new EV3UltrasonicSensor(SensorPort.S4);
    static SampleProvider distance = ultrasonicSensor.getDistanceMode();
    static float[] sample = new float[distance.sampleSize()];
    static float dangerDist = 0.20f;
    static int baseSpeed = 200;
    public void run() {
        MotorTest mControl = new MotorTest();
        System.out.println("Simple Object Avoidance Started.");
        LCD.clear();
        LCD.drawString("Program START", 0, 0);

        while (!Button.ESCAPE.isDown()) {
             distance.fetchSample(sample, 0);
             float currentDistance = sample[0];

             LCD.drawString("Dist: " + String.format("%.2f", currentDistance) + "m  ", 0, 5);

             if (currentDistance < dangerDist && currentDistance > 0) {
                 LCD.drawString("Avoiding...      ", 0, 4);
                 mControl.StopMotors();
                 mControl.Forward();

                 float turnCheckDistance;
                 float safetyMargin = 0.10f;
                 do {
                     Delay.msDelay(20);
                     distance.fetchSample(sample, 0);
                     turnCheckDistance = sample[0];
                 } while (turnCheckDistance < (dangerDist + safetyMargin) && turnCheckDistance >= 0 && !Button.ESCAPE.isDown());

                 mControl.StopMotors();
                 LCD.clear(4);

             } else {
                 LCD.drawString("Moving Forward   ", 0, 4);
                 if (!Motor.C.isMoving() || !Motor.D.isMoving()) {
                      mControl.Forward();;
                 }
             }

             Delay.msDelay(50);
         }

         mControl.StopMotors();
         ultrasonicSensor.close();
         LCD.clear();
         System.out.println("Program exiting.");
     }
 }
