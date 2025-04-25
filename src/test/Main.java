package test;

import lejos.robotics.RegulatedMotor; // Port D: Left motor | Port C: Right motor
import lejos.hardware.motor.Motor;
import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;
import lejos.utility.Delay;

public class Main {   
        
    public static void main(String[] args) {
        // Initialize classes for the threads
        MotorTest mControl = new MotorTest();
        ObjectAvoid sonic = new ObjectAvoid();
        LineFollower line = new LineFollower();
        sonic.start();
        mControl.start();
        line.start();
        
        Button.waitForAnyPress();

        }

    }
    
    


