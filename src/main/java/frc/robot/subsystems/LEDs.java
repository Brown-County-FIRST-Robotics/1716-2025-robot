package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.utils.PeriodicRunnable;

public class LEDs extends PeriodicRunnable {
  AddressableLED leds;
  AddressableLEDBuffer ledBuff;

  public LEDs() {
    super(); // Super call adds it to the registry, which calls the periodic method every tick
    leds = new AddressableLED(5);
    ledBuff = new AddressableLEDBuffer(30); // something around 280 length for LED
    leds.setLength(ledBuff.getLength());
    leds.setData(ledBuff);
    leds.start();
    // resetLeds();

    // for (var i = 0; i != ledBuff.getLength(); i++) {
    //   raindrop[i] = random.nextInt(255);
    // }
  }

  @Override
  public void periodic() {

     for (var i = 0; i < ledBuff.getLength(); i++) {
      // Sets the specified LED to the HSV values for red
      ledBuff.setHSV(i, 0, 100, 100);
    }
    //leds.setData(ledBuff);
    // leds.setData(ledBuff);
    


    LEDPattern red = LEDPattern.solid(Color.kRed);
    red.applyTo(ledBuff);
    leds.setData(ledBuff);
  }
}
