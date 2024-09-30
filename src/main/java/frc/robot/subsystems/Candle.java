// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Goals;

public class Candle extends SubsystemBase {
  private static final CANdle m_candle = new CANdle(Constants.CANids.CANdleID, Constants.CANivore);

  // Team colors
  public static final Color orange = new Color(255, 25, 0);
  public static final Color black = new Color(0, 0, 0);

  // Game piece colors
  public static final Color yellow = new Color(242, 60, 0);
  public static final Color purple = new Color(184, 0, 185);

  // Indicator colors
  public static final Color white = new Color(255, 230, 220);
  public static final Color green = new Color(56, 209, 0);
  public static final Color blue = new Color(8, 32, 255);
  public static final Color red = new Color(255, 0, 0);

  public enum AnimationTypes {
    ColorFlow,
    Fire,
    Larson,
    Rainbow,
    RgbFade,
    SingleFade,
    Strobe,
    Twinkle,
    TwinkleOff,
    SetAll
  }

  public Candle() {

    CANdleConfiguration configAll = new CANdleConfiguration();
    configAll.statusLedOffWhenActive = true;
    configAll.disableWhenLOS = false;
    configAll.stripType = LEDStripType.GRB;
    configAll.brightnessScalar = 1.0;
    configAll.vBatOutputMode = VBatOutputMode.Modulated;
    m_candle.configAllSettings(configAll, 100);
  }

  public void configBrightness(double percent) {
    m_candle.configBrightnessScalar(percent, 0);
  }

  public Command defaultCommand() {
    return runOnce(
        () -> {
          LEDSegment.BatteryIndicator.fullClear();
          LEDSegment.Blank1Indicator.fullClear();
          LEDSegment.Aim1EncoderIndicator.fullClear();
          LEDSegment.Aim2EncoderIndicator.fullClear();
          LEDSegment.Blank1Indicator.fullClear();

          LEDSegment.GoalStripR.setColor(green);
        });
  }

  public Command clearSegmentCommand(LEDSegment segment) {
    return runOnce(
        () -> {
          segment.clearAnimation();
          segment.disableLEDs();
        });
  }

  public static enum LEDSegment {
    BatteryIndicator(0, 2, 0),
    Blank1Indicator(2, 2, 1),
    Aim1EncoderIndicator(4, 1, -1),
    Aim2EncoderIndicator(5, 1, -1),
    Blank2Indicator(6, 1, -1),
    DriverStationIndicator(7, 1, -1),
    GoalStripR(8, 11, 4),
    GoalStripL(69, 6, 3),
    TopStripR(19, 40, 2),
    TopStripL(75, 40, 5);

    public final int startIndex;
    public final int segmentSize;
    public final int animationSlot;

    private LEDSegment(int startIndex, int segmentSize, int animationSlot) {
      this.startIndex = startIndex;
      this.segmentSize = segmentSize;
      this.animationSlot = animationSlot;
    }

    public void setColor(Color color) {
      clearAnimation();
      m_candle.setLEDs(color.red, color.green, color.blue, 0, startIndex, segmentSize);
    }

    private void setAnimation(Animation animation) {
      m_candle.animate(animation, animationSlot);
    }

    public void fullClear() {
      clearAnimation();
      disableLEDs();
    }

    public void clearAnimation() {
      m_candle.clearAnimation(animationSlot);
    }

    public void disableLEDs() {
      setColor(black);
    }

    public void setFlowAnimation(Color color, double speed, Direction direction) {
      setAnimation(
          new ColorFlowAnimation(
              color.red, color.green, color.blue, 0, speed, segmentSize, direction, startIndex));
    }

    public void setFadeAnimation(Color color, double speed) {
      setAnimation(
          new SingleFadeAnimation(
              color.red, color.green, color.blue, 0, speed, segmentSize, startIndex));
    }

    public void setBandAnimation(Color color, double speed) {
      setAnimation(
          new LarsonAnimation(
              color.red,
              color.green,
              color.blue,
              0,
              speed,
              segmentSize,
              BounceMode.Front,
              3,
              startIndex));
    }

    public void setStrobeAnimation(Color color, double speed) {
      setAnimation(
          new StrobeAnimation(
              color.red, color.green, color.blue, 0, speed, segmentSize, startIndex));
    }

    public void setRainbowAnimation(double speed) {
      setAnimation(new RainbowAnimation(1, speed, segmentSize, false, startIndex));
    }
  }

  public static class Color {
    public int red;
    public int green;
    public int blue;

    public Color(int red, int green, int blue) {
      this.red = red;
      this.green = green;
      this.blue = blue;
    }
  }

  Timer topLightStop = new Timer();
  double time = 3;

  public void pickedUpNote() {
    LEDSegment.TopStripL.setColor(purple);
    LEDSegment.TopStripR.setColor(purple);
    topLightStop.restart();
  }

  public void readyToShoot() {
    LEDSegment.TopStripL.setColor(green);
    LEDSegment.TopStripR.setColor(green);
    topLightStop.restart();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // if (ModeController.getAutoRotateMode() == ModeController.AutoRotateMode.OFF) {
    //  LEDSegment.MainStrip.setBandAnimation(red, 1);
    // } else {
    // SmartDashboard.putString("Top LED State",
    // RobotContainer.modeController.getTopLED().toString());

    if (topLightStop.advanceIfElapsed(time)) {

      LEDSegment.GoalStripL.setColor(black);
      LEDSegment.GoalStripR.setColor(black);
      topLightStop.stop();
    }

    switch (Goals.getGoalInfo().goal) {
      case SPEEKER:
        LEDSegment.GoalStripL.setColor(green);
        LEDSegment.GoalStripR.setColor(green);

        break;

      case FEED:
        LEDSegment.GoalStripL.setColor(white);
        LEDSegment.GoalStripR.setColor(white);

        break;

      case INTAKE:
        LEDSegment.GoalStripL.setColor(orange);
        LEDSegment.GoalStripR.setColor(orange);

        break;
      case AMP:
        LEDSegment.GoalStripL.setColor(blue);
        LEDSegment.GoalStripR.setColor(blue);

        break;
      default:
        LEDSegment.GoalStripL.setColor(black);
        LEDSegment.GoalStripR.setColor(black);
        break;
    }

    // }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
