package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.NavigableMap;
import java.util.TreeMap;
import java.util.random.RandomGenerator.LeapableGenerator;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.utils.PeriodicRunnable;

public class LEDs extends PeriodicRunnable {
  AddressableLED leds;
  AddressableLEDBuffer ledBuff;
  LEDPattern currPattern=LEDPattern.rainbow(100,40);
  List<Pair<Integer, LEDPattern>> patterns = new ArrayList<Pair<Integer, LEDPattern>>();
  static LEDs le=new LEDs();
  public static LEDs getInst(){
    return le;
  }
  public LEDs() {
    super(); // Super call adds it to the registry, which calls the periodic method every tick
    leds = new AddressableLED(5);
    ledBuff = new AddressableLEDBuffer(5); // something around 280 length for LED
    leds.setLength(ledBuff.getLength());
    leds.setData(ledBuff);
    leds.start();
    // resetLeds();

    // for (var i = 0; i != ledBuff.getLength(); i++) {
    //   raindrop[i] = random.nextInt(255);
    // }
  }
  
  public void setAllToColor(Color color) {
    patterns.clear();
    patterns.add(new Pair<Integer, LEDPattern>(1, LEDPattern.solid(color)));
  }
  public void deSet(){
    patterns.clear();
    patterns.add(new Pair<Integer, LEDPattern>(1, LEDPattern.rainbow(100,40)));
  }
  public void flashColor(Color color, Color color2, int freq){
    patterns.clear();
    patterns.add(new Pair<Integer, LEDPattern>(freq/2, LEDPattern.solid(color)));
    patterns.add(new Pair<Integer, LEDPattern>(freq/2, LEDPattern.solid(color2)));
  }
  public void flashColor(Color color, int freq){
    flashColor(color, Color.kBlack,freq);
  }
  int time=0;

  @Override
  public void periodic() {

   /*  for (var i = 0; i < ledBuff.getLength(); i++) {
      // Sets the specified LED to the HSV values for red
      ledBuff.setHSV(i, 0, 100, 100);
    }
    //leds.setData(ledBuff);
    // leds.setData(ledBuff);
    */
    time++;
    int timemod=0;
    for (var timingdata : patterns) {
      timemod+=timingdata.getFirst();
    }
    time%=timemod;
    if(patterns.size()==1){
      currPattern=patterns.get(0).getSecond();
    }else{
      int patternSum=0;
      for (Pair<Integer,LEDPattern> pair : patterns) {
        patternSum+=pair.getFirst();
        if(time<patternSum){
          currPattern=pair.getSecond();
          break;
        }
      }
    }
    

    currPattern.applyTo(ledBuff);
   

    leds.setData(ledBuff);
    

  }
}
