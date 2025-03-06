package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.utils.PeriodicRunnable;
import java.util.Random;

public class LEDs extends PeriodicRunnable {
  public enum LEDMode {
    LINE,
    WAVE,
    FLOW,
    SOLID
  }

  AddressableLED leds;
  AddressableLEDBuffer ledBuff; // Length of LEDS on robot [85]

  Random random = new Random();
  int raindrop[] = new int[180]; // how many random numbers are we creating

  int timespeed;
  int time;
  int value; // Current LED color I think (semi-unknown use)
  int value2;
  int x; // Used to change value in stuff

  public LEDMode mode;

  Color color = Color.kWhite;

  public LEDs() {
    super(); // Super call adds it to the registry, which calls the periodic method every tick
    leds = new AddressableLED(5);
    ledBuff = new AddressableLEDBuffer(280); // Something around 280 length for full LEDs (only 85 on robot though)
    leds.setLength(ledBuff.getLength());
    leds.setData(ledBuff);
    leds.start();

   // mode = LEDMode.LINE;
    x = 1;

    for (var i = 0; i < raindrop.length; i++) {
      raindrop[i] = random.nextInt(255); // Create a random number between 0-255
    }

    for (var i = 0; i != 85; i++) {
      ledBuff.setHSV(i, 130, 255, 255);
    }
  }

  @Override
  public void periodic() {
    if (mode != LEDMode.SOLID) {
      timespeed++; // Use this to control the speed of other variables around like the color change.
      timespeed = timespeed % 2; // Higher value = slower time/speed.

      if (timespeed == 0) { // Value controls the color change speed here currently.
        value++;
      }
      value = value % x;
      if (value == x - 1) {
        value2++;
      }
      value2 = value2 % raindrop.length;





      //Twinkle twinkle mode
      for(var i = 0; i < 85; i++)
      ledBuff.setHSV(i, 140, 255, raindrop[i]- value2*3);





      if (mode == LEDMode.LINE) {
        x = 95;
        // This will create random leds
        ledBuff.setHSV(value, raindrop[value2], 255, 255);

        // Cancels the 10 LEDs behind it to give a moving effect
        if (value > 9) {
          ledBuff.setHSV(value - 10, 180, 255, 255);
        }

        // Delete the LEDs at the end so they don't stay lit
        for (var i = 85; i < 95; i++) {
          ledBuff.setHSV(i, 0, 0, 0);
          ledBuff.setHSV(0, 0, 0, 0);
        }
      }
      // Single colored LED w/ changing color goes around the loop
      else if (mode == LEDMode.WAVE) {
        x = 85;
        ledBuff.setHSV(value, raindrop[value2], 255, 255);
      }

      // LEDs flow out from the center
      // Currently broken
      else if (mode == LEDMode.FLOW) {
        x = 43;
        for (var i = value + 42; i < value + 43; i++) ledBuff.setHSV(i, raindrop[value2], 255, 255);

        for (var i = -value + 42; i < -value + 43; i++)
          ledBuff.setHSV(i, raindrop[value2], 255, 255);
      }
    } else {
      for (int i = 0; i < 280; i++) ledBuff.setLED(i, color);
    }
    leds.setData(ledBuff);
  }

  public void setColor(Color color) {
    this.color = color;
  }
}