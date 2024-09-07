package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Goals;
import frc.robot.Goals.Goal;
import frc.robot.subsystems.aim.Aim;

import static edu.wpi.first.units.Units.*;

import org.opencv.core.Mat;

public class ShooterPassive {
    
    //The avarage shooter angle when note shoot at Speeker
    private static Rotation2d ShooterAvg = new Rotation2d(Degree.of(35));

    //45 deg
    private static final Rotation2d shooterFeedAngle = new Rotation2d(Degree.of(45));

    //The avarage shooter angle when note shoot
    private static Rotation2d ShootAvgAngle = new Rotation2d(Degree.of(35 / 45));

    public void ShooterPassive(){}

    public static Command ShooterPassive(Aim aim){
        return Commands.run(
        
        () -> {

            SmartDashboard.putNumber("Shooter Avg", ShooterAvg.getDegrees());
            SmartDashboard.putNumber("Shooter avg angle", ShootAvgAngle.getDegrees());

            switch (Goals.getGoalInfo().goal) {
                case FEED:
                    
                    aim.aimShooter(shooterFeedAngle);
                    break;
            
                case SPEEKER:

                    aim.aimShooter(ShooterAvg);
                    break;

                default:

                    aim.aimShooter(ShootAvgAngle);
                    break;
            }

        }, 
        
        aim);
    }

    public static void addShootSpeekerAngle(Rotation2d angle){
        
        ShooterAvg.plus(angle).div(2);
        addShootAvgAngle(angle);
    }

    public static void addShootAvgAngle(Rotation2d angle){

        ShootAvgAngle.plus(angle).div(2);

    }



    public static int exFun(){
        //creates random number between 0 and 5 
        double exampleVar = Math.round(Math.random() * 5);

        if(exampleVar == 0){
            return 0;
        }

    }

}
