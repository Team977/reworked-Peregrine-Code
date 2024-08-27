package frc.robot.subsystems.shooter.mag;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.shooter.mag.magMotorIO.OutputMag;

public class magRollerSim implements magMotorIO {

  private static final double LOOP_PERIOD_SECS = 0.02;

  DCMotorSim magMotor = new DCMotorSim(DCMotor.getFalcon500(1), 1, 0.1);

  public OutputMag getOutput() {

    magMotor.update(LOOP_PERIOD_SECS);

    OutputMag outputMag = new OutputMag();

    outputMag.amps = magMotor.getCurrentDrawAmps();
    outputMag.speed = magMotor.getAngularVelocityRPM();
    outputMag.volts = 0;

    return outputMag;
  }

  public void setSpeed(double speed) {

    magMotor.setInputVoltage(speed * RobotController.getBatteryVoltage());
  }
  ;
}
