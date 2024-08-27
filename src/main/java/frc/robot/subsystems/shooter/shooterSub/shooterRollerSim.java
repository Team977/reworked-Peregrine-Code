package frc.robot.subsystems.shooter.shooterSub;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class shooterRollerSim implements shooterMotorIO {

  private static final double LOOP_PERIOD_SECS = 0.02;

  DCMotorSim shooterMotor = new DCMotorSim(DCMotor.getFalcon500(1), 0.5, 0.05);

  public OutputShooter getOutput() {

    shooterMotor.update(LOOP_PERIOD_SECS);

    OutputShooter outputShooter = new OutputShooter();

    outputShooter.amps = shooterMotor.getCurrentDrawAmps();
    outputShooter.speed = shooterMotor.getAngularVelocityRPM() / 60 / 2;
    outputShooter.volts = 0;

    return outputShooter;
  }

  public void setVelocity(double topRPM, double bottomRPM) {
    setSpeed(topRPM);
  }

  public void setSpeed(double speed) {

    shooterMotor.setInputVoltage(speed / 108 * RobotController.getBatteryVoltage());
  }
}
