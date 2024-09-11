package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

public class Goals {

  // goals
  public static enum Goal {
    SPEEKER,
    FEED,
    INTAKE,
    AMP,
    NONE
  }

  public static class GoalInfo {

    public Goal goal = Goal.NONE;
    public boolean AutoRotate = true;
    public boolean PassivlysSwitch = true;
  }

  public static class GoalChangeInfo {

    public boolean change;
    public Goal GoaltoChange;
  }

  public static GoalInfo goalInfo = new GoalInfo();
  public static Supplier<Pose2d> position;

  private static final double SpeekerSwitchPosX = 6;
  private static final double NoteSwitchPoseX = 11;

  private static List<Supplier<GoalChangeInfo>> goalChangeInfoSup = new ArrayList<>(0);

  public static void setPositionSuppler(Supplier<Pose2d> positionSup) {
    position = positionSup;
  }

  public static void Update() {

    SmartDashboard.putString("goal", goalInfo.goal.toString());

    for (int i = 0; i < goalChangeInfoSup.size(); i++) {
      if (goalChangeInfoSup.get(i).get().change) {
        goalInfo.goal = goalChangeInfoSup.get(i).get().GoaltoChange;
        return;
      }
    }

    if (!goalInfo.PassivlysSwitch) {
      return;
    }
    // else

    if (position.get().getX() < SpeekerSwitchPosX && goalInfo.goal != Goal.AMP) {

      goalInfo.goal = Goal.SPEEKER;
      return;
    }

    if (position.get().getX() > SpeekerSwitchPosX
        && position.get().getX() < NoteSwitchPoseX
        && goalInfo.goal != Goal.AMP) {
      goalInfo.goal = Goal.FEED;
      return;
    }

    if (position.get().getX() > NoteSwitchPoseX && goalInfo.goal != Goal.AMP) {
      goalInfo.goal = Goal.INTAKE;
      return;
    }
  }

  public static void addGoalChangeInfo(Supplier<GoalChangeInfo> newGoalChangeInfo) {
    goalChangeInfoSup.add(newGoalChangeInfo);
  }

  public static Supplier<GoalInfo> getGoalInfo_Supplier() {
    return () -> goalInfo;
  }

  public static GoalInfo getGoalInfo() {
    return goalInfo;
  }

  public static void ChangeGoal(Goal newGoal) {
    goalInfo.goal = newGoal;
  }

  public static boolean AutoRotate() {
    return goalInfo.AutoRotate;
  }

  public static boolean PassivlysSwitch() {
    return goalInfo.PassivlysSwitch;
  }

  public static void setAutoRotate(boolean on) {
    goalInfo.AutoRotate = on;
  }

  public static void setPassivlysSwitch(boolean on) {
    goalInfo.PassivlysSwitch = on;
  }

  // drive Modes
  private static enum DriveMode {
    FULL,
    NORMAL,
    SLOW
  }

  private static DriveMode driveMode = DriveMode.NORMAL;

  private static final double FullTranslationModifier = 1.0;
  private static final double FullRotationModifier = 0.70;
  private static final double NormalTranslationModifier = 0.75;
  private static final double NormalRotationModifier = 0.30;
  private static final double SlowTranslationModifier = 0.40;
  private static final double SlowRotationModifier = 0.15;

  public static DriveMode getDriveSpeedMode() {
    return driveMode;
  }

  public static double getTranslationMod() {
    switch (driveMode) {
        // full speed
      case FULL:
        return FullTranslationModifier;

        // slow mode
      case SLOW:
        return SlowTranslationModifier;

        // normal
      default:
        return NormalTranslationModifier;
    }
  }

  public static double getRotationnMod() {
    switch (driveMode) {
        // full speed
      case FULL:
        return FullRotationModifier;

        // slow mode
      case SLOW:
        return SlowRotationModifier;

        // normal
      default:
        return NormalRotationModifier;
    }
  }
}
