package frc.robot.subsystems.aim;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.subsystems.shooter.ShooterConstants;

public class aimMotorsIOSim implements aimMotorsIO {
  private static final double LOOP_PERIOD_SECS = 0.02;

  private final DCMotorSim motor =
      new DCMotorSim(
          DCMotor.getKrakenX60(2),
          ShooterConstants.kAimGearRatio,
          SingleJointedArmSim.estimateMOI(Units.inchesToMeters(10), 1.5));

  private final SingleJointedArmSim m_armSim =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60(2),
          ShooterConstants.kAimGearRatio,
          SingleJointedArmSim.estimateMOI(Units.inchesToMeters(10), 3),
          Units.inchesToMeters(10),
          -90,
          90,
          true,
          0);

  private final PIDController pid;

  public aimMotorsIOSim() {
    pid = new PIDController(25, 0, 0);
  }

  public boolean setAngle(Rotation2d angle) {

    if (!(angle.getDegrees() > -70 && angle.getDegrees() < 45)) {
      // return false;
    }

    setVolts(
        pid.calculate(m_armSim.getAngleRads(), angle.getRadians())
            * RobotController.getBatteryVoltage());

    return true;
  }
  ;

  public void setVolts(double Volts) {

    m_armSim.setInputVoltage(MathUtil.clamp(Volts, -12.0, 12.0));
  }
  ;

  public OutputAim getOutputs() {

    m_armSim.update(LOOP_PERIOD_SECS);

    OutputAim outputAim = new OutputAim();

    outputAim.Current = m_armSim.getCurrentDrawAmps();
    outputAim.Rotation = new Rotation2d(m_armSim.getAngleRads());
    outputAim.Velocity = m_armSim.getCurrentDrawAmps();
    outputAim.Volts = 0;

    return outputAim;
  }
  ;

  public void resetAimZeroPosition() {}
  ;
}
