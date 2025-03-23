// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Controls the LED strip on the robot. Patterns are implemented here,
 * and they are switched by commands.
 * @author Hale Barber (H!)
 */
public class SubsystemLeds extends SubsystemBase {

  public enum Mode {
    Error,
    Blank,
    Red,
    HueCircle,
    TransFlag,
    L1Color,
    L2Color,
    L3Color,
    L4Color,
    ElevatorMovingColor,
    OrangeFire
  }

  private void loadBufferSolidColorRGB(int r, int g, int b) {
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setRGB(i, r, g, b);
    }
  }

  private void loadBufferSolidColorHSV(int h, int s, int v) {
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setHSV(i, h, s, v);
    }
  }

  private void loadBufferHueCircle(int timer) {
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setHSV(i, (i * 3 + timer) % 180, 255, 255);
    }
  }

  private void loadBufferTransFlag(int timer) {
    final Color8Bit blue = new Color8Bit(0, 150, 255);
    final Color8Bit pink = new Color8Bit(245, 65, 175);
    final Color8Bit white = new Color8Bit(200, 200, 200);
    final Color8Bit filler = new Color8Bit(0, 0, 0);

    final Color8Bit[] pattern = new Color8Bit[] {blue, blue, pink, pink, white, white, pink, pink, blue, blue, filler, filler, filler, filler};

    for (int i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setLED(i, pattern[(i + timer / 50) % pattern.length]);
    }
  }

  private void loadBufferBlank(int timer) {
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setRGB(i, 0, 0, 0);
    }
  }

  private void loadBufferError(int timer) {
    final Color8Bit yellow = new Color8Bit(255, 255, 0);
    final Color8Bit green = new Color8Bit(0, 120, 20);

    for (int i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setLED(i, (i + timer / 50) % 2 == 0 ? yellow : green);
    }
  }

  private void loadBufferFire(int timer) {
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setLED(i, getFireColorFromTimer(timer - i));
    }
  }

  private Color8Bit getFireColorFromTimer(int timer) {
    final int[] cycleCounts = new int[]{3, 5, 7, 11, 13, 17};
    final double[] amplitudes = new double[]{-0.369, -0.438, 0.177, 0.329, -0.427, -0.472};
    final int repeatTime = 50 * 5;
    final double mr = 0.1086;
    final double br = 239.6;
    final double mg = 0.9133;
    final double bg = 112.67;

    double t = 120;

    for (int i = 0; i < cycleCounts.length; i++) {
      t += Math.sin(2 * Math.PI * cycleCounts[i] * timer / ((double) repeatTime)) * amplitudes[i];
    }

    double r = br + t * mr;
    double g = bg + t * mg;
    double b = t;

    return new Color8Bit((int) r, (int) g, (int) b);
  }

  private AddressableLED led;
  private AddressableLEDBuffer ledBuffer;
  private Mode state = Mode.Error;
  private int timer = 0;
  /** Creates a new SubsystemLeds. */
  public SubsystemLeds() {
    led = new AddressableLED(4); // TODO move things to constants
    ledBuffer = new AddressableLEDBuffer(33);

    loadBufferBlank(0);

    led.setLength(ledBuffer.getLength());
    led.setData(ledBuffer);
  }

  @Override
  public void periodic() {

    if (timer % 2 == 0) {
      switch (state) {
        case Error:
          loadBufferError(timer);
          break;
        case Blank:
          loadBufferSolidColorRGB(0, 0, 0);
          break;
        case HueCircle:
          loadBufferHueCircle(timer);
          break;
        case Red:
          loadBufferSolidColorRGB(255, 0, 0);
          break;
        case TransFlag:
          loadBufferTransFlag(timer);
          break;
        case L1Color:
          loadBufferSolidColorHSV(255, 255, 200);
          break;
        case L2Color:
          loadBufferSolidColorHSV(205, 255, 200);
          break;
        case L3Color:
          loadBufferSolidColorHSV(155, 255, 200);
          break;
        case L4Color:
          loadBufferSolidColorHSV(105, 255, 200);
          break;
        case ElevatorMovingColor:
          loadBufferSolidColorHSV(0, 128, 120);
          break;
        case OrangeFire:
          loadBufferFire(timer);
      }

      led.setData(ledBuffer);
      led.start();
    }

    timer++;
  }

  public void setState(Mode state) {
    this.state = state;
  }
}
