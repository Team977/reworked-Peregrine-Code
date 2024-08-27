package frc.robot.subsystems.shooter.mag;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants;
import frc.robot.subsystems.MotorIDConst;
import frc.robot.subsystems.shooter.mag.magMotorIO.OutputMag;

public class magMotorFalcon implements magMotorIO {

  private final TalonFX leaderMagFalcon =
      new TalonFX(MotorIDConst.ShooterleaderMagFalconID, Constants.CANivore);
  private final TalonFX followerMagFalcon =
      new TalonFX(MotorIDConst.ShooterfollowerMagFalconID, Constants.CANivore);

  private final StatusSignal<Double> leaderMagPosition = leaderMagFalcon.getPosition();
  private final StatusSignal<Double> leaderMagVelocity = leaderMagFalcon.getVelocity();
  private final StatusSignal<Double> leaderMagAppliedVolts = leaderMagFalcon.getMotorVoltage();
  private final StatusSignal<Double> leaderMagCurrent = leaderMagFalcon.getStatorCurrent();
  private final StatusSignal<Double> followerMagCurrent = followerMagFalcon.getStatorCurrent();

  /** Creates a new Shooter. */
  public magMotorFalcon() {

    var magConfig = new TalonFXConfiguration();
    magConfig.CurrentLimits.StatorCurrentLimit = 40.0;
    magConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    magConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    magConfig.Slot0.kP = .5;
    leaderMagFalcon.getConfigurator().apply(magConfig);
    followerMagFalcon.getConfigurator().apply(magConfig);
    followerMagFalcon.setControl(new Follower(leaderMagFalcon.getDeviceID(), true));

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        leaderMagPosition,
        leaderMagVelocity,
        leaderMagAppliedVolts,
        leaderMagCurrent,
        followerMagCurrent);
  }

  public OutputMag getOutput() {

    OutputMag outputMag = new OutputMag();

    outputMag.amps = leaderMagCurrent.getValueAsDouble();
    outputMag.speed = leaderMagVelocity.getValueAsDouble();
    outputMag.volts = leaderMagAppliedVolts.getValueAsDouble();

    return outputMag;
  }

  public void setSpeed(double speed) {

    leaderMagFalcon.setControl(new DutyCycleOut(speed, true, false, false, false));
  }
  ;
}
