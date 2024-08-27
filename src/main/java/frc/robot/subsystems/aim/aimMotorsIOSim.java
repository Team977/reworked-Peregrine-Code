package frc.robot.subsystems.aim;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.shooter.ShooterConstants;

public class aimMotorsIOSim implements aimMotorsIO {
  private static final double LOOP_PERIOD_SECS = 0.02;

  private final DCMotorSim motor =
      new DCMotorSim(DCMotor.getKrakenX60(2), ShooterConstants.kAimGearRatio, 5);

  private final PIDController pid;

  public aimMotorsIOSim() {
    pid = new PIDController(2, 0, 0);
  }

  public boolean setAngle(Rotation2d angle) {

    if (!(angle.getDegrees() > -70 && angle.getDegrees() < 45)) {
      // return false;
    }

    setVolts(pid.calculate(motor.getAngularPositionRotations(), angle.getRotations()));

    return true;
  }
  ;

  public void setVolts(double Volts) {

    motor.setInputVoltage(MathUtil.clamp(Volts, -12.0, 12.0));
  }
  ;

  public OutputAim getOutputs() {

    motor.update(LOOP_PERIOD_SECS);

    OutputAim outputAim = new OutputAim();

    outputAim.Current = motor.getCurrentDrawAmps();
    outputAim.Rotation = new Rotation2d(motor.getAngularPositionRad());
    outputAim.Velocity = motor.getAngularVelocityRPM();
    outputAim.Volts = 0;

    return outputAim;
  }
  ;

  public void resetAimZeroPosition() {}
  ;
}
