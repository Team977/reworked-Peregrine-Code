// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake.feedIntake;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FeedIntake extends SubsystemBase {

  private static final VictorSPX FeedFrontRollers_REAL = new VictorSPX(1);
  private static final DCMotorSim FeedFrontRollers_SIM =
      new DCMotorSim(DCMotor.getMiniCIM(1), 1, .2);

  Constants.Mode mode = Constants.currentMode;

  /** Creates a new FeedIntake. */
  public FeedIntake() {}

  public void setSpeed(double speed) {
    if (mode == Constants.Mode.SIM) {
      FeedFrontRollers_SIM.setInput(speed * RobotController.getBatteryVoltage());
      return;
    }

    FeedFrontRollers_REAL.set(VictorSPXControlMode.PercentOutput, speed);
  }

  @Override
  public void periodic() {

    if (mode != Constants.Mode.SIM) {
      return;
    }

    FeedFrontRollers_SIM.update(0.02);
    SmartDashboard.putNumber(
        "Feed Front Intake Vel SIM", FeedFrontRollers_SIM.getAngularVelocityRPM());
  }
}
