package test;

import lejos.hardware.lcd.LCD;
import lejos.utility.Delay;
import lejos.hardware.Button;
import lejos.hardware.motor.Motor;

/**
 * ObjectAvoid thread handles obstacle avoidance maneuvers when signaled by Main.
 * <p>
 * Upon activation, it performs a left turn, forward movement, and right turn,
 * then resets the avoidance flag and waits for the next signal.
 * </p>
 */
public class ObjectAvoid extends Thread {

    /** Base speed for avoidance maneuvers in degrees per second. */
    private static final int baseSpeed = 200;
    /** Duration to move forward during avoidance in milliseconds. */
    private static final int durationForward = 1500;
    /** Duration to turn during avoidance in milliseconds. */
    private static final int durationTurn = 700;

    /**
     * Performs a left pivot turn by spinning the motors in opposite directions.
     */
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

    /**
     * Moves the robot straight forward for the specified duration.
     */
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

    /**
     * Performs a right pivot turn by spinning the motors in opposite directions.
     */
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

    /**
     * Main execution loop of the ObjectAvoid thread.
     * <p>
     * Waits for the isAvoiding flag from Main to become true, then executes
     * the avoidance sequence: left turn, forward move, right turn. After completion,
     * it clears the flag and displays status on the LCD. The loop exits when the ESCAPE button is pressed.
     * </p>
     * @see Main#isAvoiding
     */
    @Override
    public void run() {
        LCD.drawString("ObjectAvoid ready", 0, 3);
        // Run until user requests exit
        while (!Button.ESCAPE.isDown()) {

            // Wait for avoidance signal
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

            // Reset avoidance flag
            Main.isAvoiding.set(false);
        }
        LCD.drawString("ObjectAvoid stopped", 0, 3);
    }
}
