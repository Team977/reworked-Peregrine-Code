package frc.robot.subsystems.intake.IntakeSub;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.intake.IntakeSub.IntakeMotorsIO.OutputIntake;

public class IntakeMotorsIOSim implements IntakeMotorsIO {

  private static final double LOOP_PERIOD_SECS = 0.02;

  DCMotorSim topMotor = new DCMotorSim(DCMotor.getMiniCIM(1), 1, 0.1);
  DCMotorSim bottomMotor = new DCMotorSim(DCMotor.getMiniCIM(1), 1, 0.1);

  public IntakeMotorsIOSim() {
    SmartDashboard.putBoolean("Has Note", false);
  }

  public OutputIntake getOutputs() {

    topMotor.update(LOOP_PERIOD_SECS);
    bottomMotor.update(LOOP_PERIOD_SECS);

    OutputIntake outputIntake = new OutputIntake();

    outputIntake.speed = topMotor.getAngularVelocityRPM();

    return outputIntake;
  }

  public void setSpeed(double speed) {

    topMotor.setInputVoltage(speed * RobotController.getBatteryVoltage());
    bottomMotor.setInputVoltage(speed * RobotController.getBatteryVoltage());
  }

  public boolean HasNote() {
    return SmartDashboard.getBoolean("Has Note", false);
  }
}
