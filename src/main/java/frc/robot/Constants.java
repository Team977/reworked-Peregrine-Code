// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class CANids {
    public static final int CANdleID = 23;
  }

  public static final String CANivore = "CanivoreBus";

  public static class Vision {

    public static final String kAprilTagCameraNameLeft = "LeftAT";
    public static final String kAprilTagCameraNameRight = "RightAT";
    public static final String kNoteCameraName = "ObjDet";

    // Cam mounted facing forward, half a meter forward of center, half a meter up from center.
    public static final Transform3d kRobotToApriltagCamLeft =
        new Transform3d(
            new Translation3d(-0.3683, -0.254, 0.1524),
            new Rotation3d(0.0, -0.523599, -0.0698132 + Math.PI)); // yaw3.14

    public static final Transform3d kRobotToApriltagCamRight =
        new Transform3d(
            new Translation3d(-0.3683, 0.2540, 0.1524),
            new Rotation3d(0.0, -0.523599, 0.0698132 + Math.PI)); // yaw3.14

    // The layout of the AprilTags on the field
    public static final AprilTagFieldLayout kTagLayout =
        AprilTagFields.kDefaultField.loadAprilTagLayoutField();

    // The standard deviations of our vision estimated poses, which affect correction rate
    // (Fake values. Experiment and determine estimation noise on an actual robot.)
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(2, 2, 4);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.25, 0.25, .5);

    public static final double heaght = 110;
    public static final Translation3d SpeekerRed =
        new Translation3d(16.579342, Units.inchesToMeters(218.42), Units.inchesToMeters(heaght));
    public static final Translation3d SpeekerBlue =
        new Translation3d(-0.0381, Units.inchesToMeters(218.42), Units.inchesToMeters(heaght));
  }

  public static final Mode currentMode = Mode.SIM;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static class Drive {}
}
