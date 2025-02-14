package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.utils.PeriodicRunnable;

public class LEDs extends PeriodicRunnable {
  AddressableLED leds;
  AddressableLEDBuffer ledBuff;
  int timespeed;
  int time;
  int value; //COLOR

  public LEDs() {
    super(); // Super call adds it to the registry, which calls the periodic method every tick
    leds = new AddressableLED(5);
    ledBuff = new AddressableLEDBuffer(280); // something around 280 length for LED
    leds.setLength(ledBuff.getLength());
    leds.setData(ledBuff);
    leds.start();
    // resetLeds();

    // for (var i = 0; i != ledBuff.getLength(); i++) {
    //   raindrop[i] = random.nextInt(255);
    // }
    

    for (var i = 0; i != 280; i++) {
    //  ledBuff.setHSV(i, 0, 0, 0);
    }
  }






  @Override
  public void periodic() {

    timespeed++; //use this to control the speed of other variables around like the color change.
    timespeed = timespeed % 3; //higher value = slower time/speed.


    if(timespeed == 0){ //value controls the color change speed here currently.
      value++;
    }
  value = value % 180; 

     for (var i = 0; i < ledBuff.getLength(); i++) { //this for loop can be used to set the whole led strip to one specific color
      // Sets the specified LED to the HSV values for red
     // ledBuff.setHSV(i, 180, 255, 255);
    }

    for (var i = 100; i < 110; i++) {  //this causes the leds to move around 
      ledBuff.setHSV(i, value, 255, 255);
    }
    for (var i = 138; i < 150; i++) {  //this causes the leds to move around 
      ledBuff.setHSV(i, value, 255, 255);
    }

    

    

     //ledBuff.setHSV(30, 60, 255, 255);
     leds.setData(ledBuff);
    


    /*LEDPattern red = LEDPattern.solid(Color.kRed);
    red.applyTo(ledBuff);
    leds.setData(ledBuff); */
  }
}
