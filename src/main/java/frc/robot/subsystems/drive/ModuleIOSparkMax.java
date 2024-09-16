// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volt;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

/*
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
 */

/**
 * Module IO implementation for SparkMax drive motor controller, SparkMax turn motor controller (NEO
 * or NEO 550), and analog absolute encoder connected to the RIO
 *
 * <p>NOTE: This implementation should be used as a starting point and adapted to different hardware
 * configurations (e.g. If using a CANcoder, copy from "ModuleIOTalonFX")
 *
 * <p>To calibrate the absolute encoder offsets, point the modules straight (such that forward
 * motion on the drive motor will propel the robot forward) and copy the reported values from the
 * absolute encoders using AdvantageScope. These values are logged under
 * "/Drive/ModuleX/TurnAbsolutePositionRad"
 */
public class ModuleIOSparkMax implements ModuleIO {

  // Gear ratios for SDS MK4i L2, adjust as necessary
  private static final double DRIVE_GEAR_RATIO = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
  private static final double TURN_GEAR_RATIO = 150.0 / 7.0;

  private final CANSparkMax driveSparkMax;
  private final CANSparkMax turnSparkMax;

  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder turnRelativeEncoder;
  // private final AnalogInput turnAbsoluteEncoder;
  private final CANcoder cancoder;
  private final StatusSignal<Double> turnAbsolutePosition;

  private final boolean isTurnMotorInverted = true;
  private final Rotation2d absoluteEncoderOffset;

  public ModuleIOSparkMax(int index) {

    switch (index) {

        // Front Left
      case 0:
        driveSparkMax = new CANSparkMax(1, MotorType.kBrushless);
        turnSparkMax = new CANSparkMax(2, MotorType.kBrushless);
        cancoder = new CANcoder(3);
        // turnAbsoluteEncoder = new AnalogInput(0);
        absoluteEncoderOffset = new Rotation2d(-0.077881 * 2 * Math.PI); // MUST BE CALIBRATED
        break;

        // Front Right
      case 1:
        driveSparkMax = new CANSparkMax(4, MotorType.kBrushless);
        turnSparkMax = new CANSparkMax(5, MotorType.kBrushless);
        cancoder = new CANcoder(6);
        absoluteEncoderOffset = new Rotation2d(0.283936 * 2 * Math.PI); // MUST BE CALIBRATED
        break;

        // Back Left
      case 2:
        driveSparkMax = new CANSparkMax(7, MotorType.kBrushless);
        turnSparkMax = new CANSparkMax(8, MotorType.kBrushless);
        cancoder = new CANcoder(9);
        absoluteEncoderOffset = new Rotation2d(0.098633 * 2 * Math.PI); // MUST BE CALIBRATED
        break;

        // Back Right
      case 3:
        driveSparkMax = new CANSparkMax(10, MotorType.kBrushless);
        turnSparkMax = new CANSparkMax(11, MotorType.kBrushless);
        cancoder = new CANcoder(12);
        absoluteEncoderOffset = new Rotation2d(-0.145020 * 2 * Math.PI); // MUST BE CALIBRATED
        break;

      default:
        throw new RuntimeException("Invalid module index");
    }

    driveSparkMax.restoreFactoryDefaults();
    turnSparkMax.restoreFactoryDefaults();

    driveSparkMax.setCANTimeout(250);
    turnSparkMax.setCANTimeout(+250);

    driveEncoder = driveSparkMax.getEncoder();
    turnRelativeEncoder = turnSparkMax.getEncoder();

    turnSparkMax.setInverted(isTurnMotorInverted);
    driveSparkMax.setSmartCurrentLimit(40);
    turnSparkMax.setSmartCurrentLimit(30);
    driveSparkMax.enableVoltageCompensation(12.0);
    turnSparkMax.enableVoltageCompensation(12.0);

    driveEncoder.setPosition(0.0);
    driveEncoder.setMeasurementPeriod(10);
    driveEncoder.setAverageDepth(2);

    turnRelativeEncoder.setPosition(0.0);
    turnRelativeEncoder.setMeasurementPeriod(10);
    turnRelativeEncoder.setAverageDepth(2);

    driveSparkMax.setCANTimeout(0);
    turnSparkMax.setCANTimeout(0);

    driveSparkMax.burnFlash();
    turnSparkMax.burnFlash();

    // New CANcoder add on
    cancoder.getConfigurator().apply(new CANcoderConfiguration());

    turnAbsolutePosition = cancoder.getAbsolutePosition();

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, turnAbsolutePosition);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {

    BaseStatusSignal.refreshAll(turnAbsolutePosition);

    inputs.drivePositionRad =
        Units.rotationsToRadians(driveEncoder.getPosition()) / DRIVE_GEAR_RATIO;

    inputs.driveVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(driveEncoder.getVelocity()) / DRIVE_GEAR_RATIO;

    inputs.driveAppliedVolts = driveSparkMax.getAppliedOutput() * driveSparkMax.getBusVoltage();

    inputs.driveCurrentAmps = new double[] {driveSparkMax.getOutputCurrent()};

    /*
    inputs.turnAbsolutePosition =
        new Rotation2d(
                turnAbsoluteEncoder.getVoltage() / RobotController.getVoltage5V() * 2.0 * Math.PI)
            .minus(absoluteEncoderOffset);
      */

    inputs.turnAbsolutePosition =
        Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble())
            .minus(absoluteEncoderOffset);

    inputs.turnPosition =
        Rotation2d.fromRotations(turnRelativeEncoder.getPosition() / TURN_GEAR_RATIO);

    inputs.turnVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(turnRelativeEncoder.getVelocity())
            / TURN_GEAR_RATIO;

    inputs.turnAppliedVolts = turnSparkMax.getAppliedOutput() * turnSparkMax.getBusVoltage();

    inputs.turnCurrentAmps = new double[] {turnSparkMax.getOutputCurrent()};
  }

  @Override
  public LogOut getLog() {
    LogOut log = new LogOut();
    log.rotation = Rotations.of(driveEncoder.getPosition());
    log.anglerVelocity = RotationsPerSecond.of(driveEncoder.getVelocity());
    log.amps = (Amps.of(driveSparkMax.getOutputCurrent()));
    log.volts = (Volt.of(driveSparkMax.getAppliedOutput() * driveSparkMax.getBusVoltage()));
    log.Position =
        (Meters.of(
            driveEncoder.getPosition() / DRIVE_GEAR_RATIO * Math.PI * Units.inchesToMeters(4.0)));
    log.Velocity =
        (MetersPerSecond.of(
            driveEncoder.getVelocity()
                / 60
                / DRIVE_GEAR_RATIO
                * Math.PI
                * Units.inchesToMeters(4.0)));
    return log;
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveSparkMax.setVoltage(volts);
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnSparkMax.setVoltage(volts);
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    driveSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }

  @Override
  public void setTurnBrakeMode(boolean enable) {
    turnSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }
}
