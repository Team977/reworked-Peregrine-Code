package frc.robot.subsystems.intake.IntakeSub;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.AnalogInput;
import frc.robot.subsystems.MotorIDConst;

public class IntakeMotorsIOVictor implements IntakeMotorsIO {

  private final AnalogInput IRsensor;
  private final VictorSPX frontRollers;
  private final VictorSPX backRollers;

  /** Creates a new Intake. */
  public IntakeMotorsIOVictor() {
    IRsensor = new AnalogInput(IntakeConstants.IRsensorPort);
    IRsensor.setAverageBits(4);
    frontRollers = new VictorSPX(MotorIDConst.IntakefrontRollersID);
    backRollers = new VictorSPX(MotorIDConst.IntakeBackRollersID);
  }

  public void setSpeed(double speed) {
    frontRollers.set(ControlMode.PercentOutput, speed);
    backRollers.set(ControlMode.PercentOutput, speed);
  }

  public OutputIntake getOutputs() {
    OutputIntake outputIntake = new OutputIntake();

    return outputIntake;
  }

  public boolean HasNote() {

    return IRsensor.getAverageValue() > IntakeConstants.IRthreshold;
  }
  ;
}
