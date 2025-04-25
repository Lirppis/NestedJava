package test;

import lejos.hardware.lcd.LCD;
import lejos.utility.Delay;
import lejos.hardware.Button;
import lejos.hardware.motor.Motor;

public class ObjectAvoid extends Thread {
    int turnamount = 0;
    private static final int baseSpeed = 200;
    private static final int durationForward = 2000;
    private static final int durationTurn = 200;
    

    private void TurnLeft() {
        ObjectAvoid turn = new ObjectAvoid();
        Motor.C.setSpeed(baseSpeed);
        Motor.D.setSpeed(baseSpeed);
        Motor.C.forward();
        Motor.D.backward();
        Delay.msDelay(durationTurn);
        Motor.D.stop(true);
        Motor.C.stop(true);
        Delay.msDelay(100);
        turn.turnamount += 200;
        Motor.C.setSpeed(baseSpeed);
        Motor.D.setSpeed(baseSpeed);
        Motor.C.forward();
        Motor.D.backward();
        Delay.msDelay(durationTurn / 2);
        if (Main.getDistance() < Main.DANGER_DISTANCE){
            TurnLeft();
        }
    }

    private void MoveForward() {
        LCD.drawString("Going forward", 0, 4);
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
        ObjectAvoid turn = new ObjectAvoid();
        //LCD.drawString("turning right", 0, 5);
        LCD.drawString("amount" + turnamount, 0, 5);
        Motor.C.setSpeed(baseSpeed);
        Motor.D.setSpeed(baseSpeed);
        Motor.C.backward();
        Motor.D.forward();
        Delay.msDelay(turn.turnamount);
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
