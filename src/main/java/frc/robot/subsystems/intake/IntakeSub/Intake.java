// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake.IntakeSub;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.intake.IntakeSub.IntakeMotorsIO.OutputIntake;

public class Intake extends SubsystemBase {

  IntakeMotorsIO intakeMotorsIO;

  double DesiredSpeed;

  /** Creates a new Intake. */
  public Intake(IntakeMotorsIO intakeMotorsIO) {

    this.intakeMotorsIO = intakeMotorsIO;
  }

  @Override
  public void periodic() {

    OutputIntake outputIntake = intakeMotorsIO.getOutputs();
    SmartDashboard.putNumber("intake Sim Speed", outputIntake.speed);
    SmartDashboard.putBoolean("Intake has Note", isNotePresent());
    SmartDashboard.putNumber("intake Desired Speed", DesiredSpeed);

    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("Voltage", IRsensor.getVoltage());
    // SmartDashboard.putNumber("AverageVoltage", IRsensor.getAverageVoltage());
    // SmartDashboard.putNumber("Value", IRsensor.getValue());
    // SmartDashboard.putNumber("AverageValue", IRsensor.getAverageValue());
    // SmartDashboard.putBoolean("Note Present", isNotePresent());
  }

  public void intakeNote(double speed) {
    intakeMotorsIO.setSpeed(speed);
    DesiredSpeed = speed;
  }

  public boolean isNotePresent() {
    return intakeMotorsIO.HasNote();
  }

  public Trigger NoteSensor(){
    return new Trigger(() -> isNotePresent());
  }
}
