package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants;
import frc.robot.subsystems.MotorIDConst;

public class shooterMotorFalcon implements shooterMotorIO {

  private final TalonFX topShooterFalcon =
      new TalonFX(MotorIDConst.ShooterTopShooterFalconID, Constants.CANivore);
  private final TalonFX bottomShooterFalcon =
      new TalonFX(MotorIDConst.ShooterBottomShooterFalconID, Constants.CANivore);

  private final StatusSignal<Double> topShootPosition = topShooterFalcon.getPosition();
  private final StatusSignal<Double> topShootVelocity = topShooterFalcon.getVelocity();
  private final StatusSignal<Double> topShootAppliedVolts = topShooterFalcon.getMotorVoltage();
  private final StatusSignal<Double> topShootCurrent = topShooterFalcon.getStatorCurrent();
  private final StatusSignal<Double> bottomShootPosition = bottomShooterFalcon.getPosition();
  private final StatusSignal<Double> bottomShootVelocity = bottomShooterFalcon.getVelocity();
  private final StatusSignal<Double> bottomShootAppliedVolts =
      bottomShooterFalcon.getMotorVoltage();
  private final StatusSignal<Double> bottomShootCurrent = bottomShooterFalcon.getStatorCurrent();

  /** Creates a new Shooter. */
  public shooterMotorFalcon() {

    var shooterConfig = new TalonFXConfiguration();
    shooterConfig.Slot0.kP = ShooterConstants.ShooterKp;
    shooterConfig.Slot0.kD = ShooterConstants.ShooterKd;
    shooterConfig.Slot0.kV = 2.0;

    // set Motion Magic Velocity settings
    var shooterMotionMagicConfigs = shooterConfig.MotionMagic;
    shooterMotionMagicConfigs.MotionMagicAcceleration =
        1600; // Target acceleration of 400 rps/s (0.25 seconds to max)
    shooterMotionMagicConfigs.MotionMagicJerk = 8000.0; // Target jerk of 4000 rps/s/s (0.1 seconds)

    shooterConfig.CurrentLimits.StatorCurrentLimit = 160.0;
    shooterConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    topShooterFalcon.setInverted(true);
    bottomShooterFalcon.setInverted(true);
    topShooterFalcon.getConfigurator().apply(shooterConfig);
    bottomShooterFalcon.getConfigurator().apply(shooterConfig);

    // bottomShooterFalcon.setControl(new Follower(topShooterFalcon.getDeviceID(), true));

    topShooterFalcon.setInverted(true);
    bottomShooterFalcon.setInverted(true);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        topShootPosition,
        topShootVelocity,
        topShootAppliedVolts,
        topShootCurrent,
        bottomShootPosition,
        bottomShootVelocity,
        bottomShootAppliedVolts,
        bottomShootCurrent);
    topShooterFalcon.optimizeBusUtilization();
    bottomShooterFalcon.optimizeBusUtilization();
  }

  public OutputShooter getOutput() {

    OutputShooter outputMag = new OutputShooter();

    topShootCurrent.refresh();
    topShootVelocity.refresh();
    topShootAppliedVolts.refresh();

    outputMag.amps = topShootCurrent.getValueAsDouble();
    outputMag.speed = topShootVelocity.getValueAsDouble();
    outputMag.volts = topShootAppliedVolts.getValueAsDouble();

    return outputMag;
  }

  public void setVelocity(double topRPM, double bottomRPM) {
    topShooterFalcon.setControl(new MotionMagicVelocityVoltage(topRPM));
    bottomShooterFalcon.setControl(new MotionMagicVelocityVoltage(bottomRPM));
  }
}
