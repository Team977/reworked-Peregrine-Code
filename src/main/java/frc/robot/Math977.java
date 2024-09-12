package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Math977 {
 
    public static Rotation3d getRotationBetweanSpeekerAndRobot(Translation3d Robot){
        Translation3d Speeker = isRed()? Constants.Vision.SpeekerRed : Constants.Vision.SpeekerBlue;
        return getRotationBetweenObjectAndRobot(Speeker, Robot);
    }

    public static Rotation2d getRotationBetweanSpeekerAndRobotPitch(Translation3d Robot){
                return new Rotation2d(getRotationBetweanSpeekerAndRobot(Robot).getY());

    }

    public static Rotation2d getRotationBetweanSpeekerAndRobotYaw(Translation3d Robot){
                        return new Rotation2d(getRotationBetweanSpeekerAndRobot(Robot).getZ());

    }

    public static Rotation3d getRotationBetweenObjectAndRobot(Translation3d object, Translation3d Robot){
        return new Rotation3d(0 , getRotationBetweanObjectAndRobotPitch(object, Robot).getRadians(), getRotationBetweanObjectAndRobotYaw(object, Robot).getRadians());
    }

    
    public static Rotation2d getRotationBetweanObjectAndRobotPitch(Translation3d object, Translation3d Robot){
        return 
            getAngleBetweenRobotAndObject
                (
                    new Translation2d(object.toTranslation2d().getDistance(Robot.toTranslation2d()), object.getZ()),
                    object.toTranslation2d());
    }

    public static Rotation2d getRotationBetweanObjectAndRobotYaw(Translation3d object, Translation3d Robot){

        return 
            getAngleBetweenRobotAndObject
                (
                    object.toTranslation2d(),
                    Robot.toTranslation2d());
    }


    /* 
     * gets the angle between a static Object and the robot
    */
    public static Rotation2d getAngleBetweenRobotAndObject(Translation2d Object, Translation2d Robot) {

        // add offset to robot Pos
        Robot.minus( Object );

        // get angle
        return new Rotation2d(Robot.getX(), Robot.getY());
    }

    public static boolean isRed() {
    return DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == Alliance.Red;
    }

}
