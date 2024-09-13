package frc.robot.subsystems.IO;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface IOMoudlue {
    
    public default DoubleSupplier getXPower() {return () -> 0;}
    public default DoubleSupplier getYPower() {return () -> 0;}
    public default DoubleSupplier getOmegaPower() {return () -> 0;}

    public default Trigger getShooterReady() { return new Trigger(() -> false);}

}
