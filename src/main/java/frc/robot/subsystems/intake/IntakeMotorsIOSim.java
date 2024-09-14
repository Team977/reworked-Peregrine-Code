package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.intake.IntakeMotorsIO.OutputIntake;

public class IntakeMotorsIOSim implements IntakeMotorsIO {

  private static final double LOOP_PERIOD_SECS = 0.02;

  DCMotorSim topMotor = new DCMotorSim(DCMotor.getMiniCIM(1), 1, 0.1);
  DCMotorSim bottomMotor = new DCMotorSim(DCMotor.getMiniCIM(1), 1, 0.1);

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
    return false;
  }
}
