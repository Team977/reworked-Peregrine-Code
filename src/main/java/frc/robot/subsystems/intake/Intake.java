// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.MotorIDConst;

public class Intake extends SubsystemBase {

  private final AnalogInput IRsensor;
  private final VictorSPX frontRollers;
  private final VictorSPX backRollers;

  /** Creates a new Intake. */
  public Intake() {
    IRsensor = new AnalogInput(IntakeConstants.IRsensorPort);
    IRsensor.setAverageBits(4);
    frontRollers = new VictorSPX(MotorIDConst.IntakefrontRollersID);
    backRollers = new VictorSPX(MotorIDConst.IntakeBackRollersID);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("Voltage", IRsensor.getVoltage());
    // SmartDashboard.putNumber("AverageVoltage", IRsensor.getAverageVoltage());
    // SmartDashboard.putNumber("Value", IRsensor.getValue());
    // SmartDashboard.putNumber("AverageValue", IRsensor.getAverageValue());
    // SmartDashboard.putBoolean("Note Present", isNotePresent());
  }

  public void intakeNote() {
    frontRollers.set(ControlMode.PercentOutput, IntakeConstants.defaultIntakeSpeed);
    backRollers.set(ControlMode.PercentOutput, IntakeConstants.defaultIntakeSpeed);
  }

  public void intakeNote(double speed) {
    frontRollers.set(ControlMode.PercentOutput, speed);
    backRollers.set(ControlMode.PercentOutput, speed);
  }

  public void stopIntake() {
    frontRollers.set(ControlMode.PercentOutput, 0);
    backRollers.set(ControlMode.PercentOutput, 0);
  }

  public boolean isNotePresent() {
    return IRsensor.getAverageValue() > IntakeConstants.IRthreshold;
  }
}
