package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.utils.PeriodicRunnable;
import java.util.Random;

public class LEDs extends PeriodicRunnable {
  AddressableLED leds;
  AddressableLEDBuffer ledBuff; //length of LEDS on robot [85]

  Random random = new Random();
  int raindrop[] = new int[180]; //how many random numbers are we creating 

  int timespeed;
  int time;
  int value; //COLOR
  int value2;
  int x; //used to change value in stuff

  boolean mode1;
  boolean mode2;

  public LEDs() {
    super(); // Super call adds it to the registry, which calls the periodic method every tick
    leds = new AddressableLED(5);
    ledBuff = new AddressableLEDBuffer(280); // something around 280 length for full LED (only 85 on robot though)
    leds.setLength(ledBuff.getLength());
    leds.setData(ledBuff);
    leds.start();

    mode1 = false; //lights moving in a line mode
    mode2 = true;
    x = 1; 

     for (var i = 0; i < 180; i++) {
       raindrop[i] = random.nextInt(255); //create a random number between 0-255 
     }
    

    for (var i = 0; i != 85; i++) {
      ledBuff.setHSV(i, 130, 255, 255);
    }
  }






  @Override
  public void periodic() {

    timespeed++; //use this to control the speed of other variables around like the color change.
    timespeed = timespeed % 3; //higher value = slower time/speed.


    if(timespeed == 0){ //value controls the color change speed here currently.
      value++;
    }
  value = value % x; 
  if(value == x - 1){
    value2++;
  }


     for (var i = 0; i < 85; i++) { //this for loop can be used to set the whole led strip to one specific color
      // Sets the specified LED to the HSV values to ALL WHITE
     // ledBuff.setHSV(i, 130, 255, 255);
    }


    if(mode1 == true) {  /////////////////////////////////////START OF MODE 1
      x = 95;
    for (var i = value; i < 1 + value; i++) {  //This will creat random leds 
      ledBuff.setHSV(i, raindrop[i], 255, 255);
    }
    
    if(value > 11) { //this will cancle thd leds 10 indexs behind giving it a moving effect
    ledBuff.setHSV(value-10, 130, 255, 50);
    }

    for(var i = 85; i < 95; i++) { //this will delete the leds at the end so they dont just stay light
    ledBuff.setHSV(i, 0, 0, 0); 
    ledBuff.setHSV(0, 0, 0, 0); 
  }
    }  /////////////////////////////////////////////////////////END OF MODE 1
    
    if(mode2 == true){ /////////////////////////////////////////START OF MODE 2
      x = 85;
        ledBuff.setHSV(value, raindrop[value2], 255, 255);
    }

   /*  if(mode3 == true){ /////////////////////////////////////////START OF MODE 3 THIS DOES NOT WORK CURRENTLY
      x = 43;
      for(var i = value + 42; i < value + 43; i++)
        ledBuff.setHSV(i, raindrop[value2], 255, 255);
      
    } */
    

     //ledBuff.setHSV(30, 60, 255, 255);
     leds.setData(ledBuff);
    

    


    /*LEDPattern red = LEDPattern.solid(Color.kRed);
    red.applyTo(ledBuff);
    leds.setData(ledBuff); */
  }
}
