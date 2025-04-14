package robot;

import handlerthreads.ColorSensorHandlerThread;
import handlerthreads.UltrasonicSensorHandlerThread;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.Motor;
import lejos.robotics.RegulatedMotor;

import java.io.File;

/**
 * Robot class represents the main control unit for the robot's actions.
 * It handles the initialization of sensors, motor control, obstacle detection, and navigation.
 */
public class Robot {
    // SensorHandlerThread objects:
    private ColorSensorHandlerThread lsht; // Color sensor handler thread
    private UltrasonicSensorHandlerThread usht; // Ultrasonic sensor handler thread

    // Robot's motors:
    private RegulatedMotor motorLeft = Motor.D; // Left motor
    private RegulatedMotor motorRight = Motor.C; // Right motor

    // Robot's speed attributes:
    private int minSpeed = 50; // Minimum speed
    private int maxSpeed = 200; // Maximum speed

    // Wheel attributes:
    // (Measurements are represented in centimeters)
    private double wheelPerimeter = Math.PI * 5.5; // Wheel perimeter
    private double wheelGap = 10.2; // Wheel gap
    // Variable to store the position relative to the line
    private boolean leftOfLine = false; // Whether the robot is left of the line

    /**
     * Constructs a Robot object.
     * Initializes the color sensor handler thread and ultrasonic sensor handler thread.
     */
    public Robot() {
        this.lsht = new ColorSensorHandlerThread();
        this.usht = new UltrasonicSensorHandlerThread();
    }

    /**
     * Retrieves the wheel perimeter.
     *
     * @return The wheel perimeter.
     */
    public double getWheelPerimeter() {
        return this.wheelPerimeter;
    }

    /**
     * Retrieves the wheel gap.
     *
     * @return The wheel gap.
     */
    public double getWheelGap() {
        return this.wheelGap;
    }

    /**
     * Sets the wheel perimeter.
     *
     * @param newWheelPerimeter The new wheel perimeter.
     * @return True if the operation was successful, false otherwise.
     */
    public boolean setWheelPerimeter(double newWheelPerimeter) {
        this.wheelPerimeter = newWheelPerimeter;
        return true;
    }

    /**
     * Sets the wheel gap.
     *
     * @param newWheelGap The new wheel gap.
     * @return True if the operation was successful, false otherwise.
     */
    public boolean setWheelGap(double newWheelGap) {
        this.wheelGap = newWheelGap;
        return true;
    }

    /**
     * Starts the robot's main control loop.
     *
     * @return True if the operation was successful, false otherwise.
     */
    public boolean start() {
        LCD.drawString("Getting started", 0, 1);
        long startTime = System.currentTimeMillis(); // Start time of the loop
        File soundtrack = new File("mario_tash.wav");

        // Start sensor threads
        this.lsht.start();
        this.usht.start();

        boolean obstacleDetectedOnce = false; // Indicates if an obstacle has been detected before

        // Main control loop
        while (!Button.ESCAPE.isDown()) {
            // Check if the robot is on the line
            if (this.lsht.isOnLine()) {
                this.motorLeft.setSpeed(this.maxSpeed);
                this.motorRight.setSpeed(this.minSpeed);
                leftOfLine = false; // Robot is on or right of the line
            } else {
                this.motorLeft.setSpeed(this.minSpeed);
                this.motorRight.setSpeed(this.maxSpeed);
                leftOfLine = true; // Robot is left of the line
            }

            // Move the robot forward
            this.motorLeft.forward();
            this.motorRight.forward();

            // Check for obstacle
            if (this.usht.detectsObstacle()) {
                // Stop the robot
                Sound.twoBeeps();
                this.motorLeft.stop();
                this.motorRight.stop();

                if (obstacleDetectedOnce) {
                    // If obstacle detected again, end the program
                    LCD.clear();
                    long endTime = System.currentTimeMillis(); // End time of the loop
                    long elapsedTime = endTime - startTime;
                    LCD.drawString("Kierrosaika: " + elapsedTime / 1000 + " sec", 0, 2);
                    Sound.buzz();
                    this.motorLeft.stop();
                    this.motorRight.stop();

                    Sound.playSample(soundtrack);
                    return true; // End the program
                } else {
                    obstacleDetectedOnce = true; // Mark that an obstacle has been detected at least once
                }

                // Check the position relative to the line
                if (leftOfLine) {
                    // If left of the line, turn right to find the line again
                    this.motorLeft.rotate(90, true);
                    this.motorRight.rotate(-90, true);

                    // Wait until the turn is complete
                    while (this.motorLeft.isMoving() || this.motorRight.isMoving()) {
                        Thread.yield();
                    }
                }

                // Turn the robot slightly to the right
                this.motorLeft.rotate(80, true);
                this.motorRight.rotate(-80, true);

                // Wait until the turn is complete
                while (this.motorLeft.isMoving() || this.motorRight.isMoving()) {
                    Thread.yield();
                }

                // Move forward 25cm
                this.motorLeft.setSpeed(this.maxSpeed);
                this.motorRight.setSpeed(this.maxSpeed);
                this.motorLeft.forward();
                this.motorRight.forward();

                // Wait until the robot has moved forward 25cm
                try {
                    Thread.sleep(2500); // Move forward for 2.5 seconds (approx. 25cm)
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }

                // Stop the robot
                this.motorLeft.stop();
                this.motorRight.stop();

                // Turn the robot slightly to the left
                this.motorLeft.rotate(-120, true);
                this.motorRight.rotate(120, true);

                // Wait until the turn is complete
                while (this.motorLeft.isMoving() || this.motorRight.isMoving()) {
                    Thread.yield();
                }

                // Move forward 23cm
                this.motorLeft.setSpeed(this.maxSpeed);
                this.motorRight.setSpeed(this.maxSpeed);
                this.motorLeft.forward();
                this.motorRight.forward();

                try {
                    Thread.sleep(2300); // Move forward for 2.3 seconds (approx. 23cm)
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }

            // Display the color detected by the color sensor
            LCD.drawString("Color: " + lsht.getRecentSample(), 0, 5);
        }

        // Exit the sensor threads
        this.lsht.exit();
        this.usht.exit();

        return true;
    }
}