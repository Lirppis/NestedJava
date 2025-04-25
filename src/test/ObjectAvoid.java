package test;

import lejos.hardware.lcd.LCD;
import lejos.utility.Delay;
import lejos.hardware.Button;
import lejos.hardware.motor.Motor;

public class ObjectAvoid extends Thread {

    private static final int baseSpeed = 200;
    private static final int durationForward = 1500;
    private static final int durationTurn = 700;

    private void TurnLeft() {
        Motor.C.setSpeed(baseSpeed);
        Motor.D.setSpeed(baseSpeed);
        Motor.C.forward();
        Motor.D.backward();
        Delay.msDelay(durationTurn);
        Motor.D.stop(true);
        Motor.C.stop(true);
        Delay.msDelay(100);
    }

    private void MoveForward() {
        Motor.C.setSpeed(baseSpeed);
        Motor.D.setSpeed(baseSpeed);
        Motor.C.forward();
        Motor.D.forward();
        Delay.msDelay(durationForward);
        Motor.D.stop(true);
        Motor.C.stop(true);
        Delay.msDelay(100);
    }

    private void TurnRight() {
        Motor.C.setSpeed(baseSpeed);
        Motor.D.setSpeed(baseSpeed);
        Motor.C.backward();
        Motor.D.forward();
        Delay.msDelay(durationTurn);
        Motor.D.stop(true);
        Motor.C.stop(true);
        Delay.msDelay(100);
    }

    @Override
    public void run() {
        LCD.drawString("ObjectAvoid ready", 0, 3);
        while (!Button.ESCAPE.isDown()) {

            while (!Main.isAvoiding.get() && !Button.ESCAPE.isDown()) {
                Delay.msDelay(50);
            }

            if (Button.ESCAPE.isDown()) {
                break;
            }
            TurnLeft();
            MoveForward();
            TurnRight();

            Motor.D.stop(true);
            Motor.C.stop(true); 
            LCD.drawString("Avoid finished   ", 0, 6);
            Delay.msDelay(100);

            Main.isAvoiding.set(false);
        }
        LCD.drawString("ObjectAvoid stopped", 0, 3);
    }
}
