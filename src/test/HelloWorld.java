package test;

import lejos.hardware.motor.Motor;
import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;
import lejos.utility.Delay;

public class HelloWorld {
    public static void main(String[] args)
    {
        String message1 = "This is my 1st LEGO code.";
        String message2 = "Make me autonomous";
        String message3 = "Press any button to Stop.";
        LCD.clear();
        LCD.drawString("Welcome", 0, 1);

        Delay.msDelay(1000);
        TextWrap(message1, 2);
        TextWrap(message2, 3);
        //LCD.drawString("This is my 1st LEGO code.", 0, 2);

        Delay.msDelay(2000);
        //LCD.drawString("Make me autonomous", 0, 4);
        TextWrap(message3, 4);
        //LCD.drawString("Press any button to stop.", 0, 6);
        
       
        // Wait for a button press to exit
        Button.waitForAnyPress();
    }

    
    public static void TextWrap(String msg, int StartY) {
        int ScreenLength = 16;
        int LocalX = 0;
        int y = StartY;

        for (int i = 0; i < msg.length(); i+= ScreenLength){
            String line = msg.substring(i, Math.min(i + ScreenLength, msg.length()));
            LCD.drawString(line,LocalX, y++);
        }

    }


}

