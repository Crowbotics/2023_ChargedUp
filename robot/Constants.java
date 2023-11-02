// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    public static final int kFrontLeftDriveMotorPort = 1;
    public static final int kRearLeftDriveMotorPort = 3;
    public static final int kFrontRightDriveMotorPort = 5;
    public static final int kRearRightDriveMotorPort = 7;

    public static final int kFrontLeftTurningMotorPort = 2;
    public static final int kRearLeftTurningMotorPort = 4;
    public static final int kFrontRightTurningMotorPort = 6;
    public static final int kRearRightTurningMotorPort = 8;

    public static final int kFrontLeftCanCoderChannel = 22;
    public static final int kRearLeftCanCoderChannel = 24;
    public static final int kFrontRightCanCoderChannel = 26;
    public static final int kRearRightCanCoderChannel = 28;

    /**
    public static final int[] kFrontLeftTurningEncoderPorts = new int[] {0, 1};
    public static final int[] kRearLeftTurningEncoderPorts = new int[] {2, 3};
    public static final int[] kFrontRightTurningEncoderPorts = new int[] {4, 5};
    public static final int[] kRearRightTurningEncoderPorts = new int[] {6, 7};
    */

    public static final boolean kFrontLeftTurningEncoderReversed = false;
    public static final boolean kRearLeftTurningEncoderReversed = true;
    public static final boolean kFrontRightTurningEncoderReversed = false;
    public static final boolean kRearRightTurningEncoderReversed = true;

    /**
    public static final int[] kFrontLeftDriveEncoderPorts = new int[] {8, 9};
    public static final int[] kRearLeftDriveEncoderPorts = new int[] {10, 11};
    public static final int[] kFrontRightDriveEncoderPorts = new int[] {12, 13};
    public static final int[] kRearRightDriveEncoderPorts = new int[] {14, 15};
    */

    public static final boolean kFrontLeftDriveEncoderReversed = false;
    public static final boolean kRearLeftDriveEncoderReversed = true;
    public static final boolean kFrontRightDriveEncoderReversed = false;
    public static final boolean kRearRightDriveEncoderReversed = true;

    public static final double kTrackWidth = 0.394;
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = 0.394;
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    public static final boolean kGyroReversed = false;

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or theoretically
    // for *your* robot's drive.
    // The SysId tool provides a convenient method for obtaining these values for your robot.
    //public static final double ksVolts = 1;
    //public static final double kvVoltSecondsPerMeter = 0.8;
    //public static final double kaVoltSecondsSquaredPerMeter = 0.15;

    public static final double kMaxSpeedMetersPerSecond = 15 * 4096;

    public static final double kFrontLeftOffset = 0;
    public static final double kFrontRightOffset = 0;
    public static final double kRearLeftOffset = 0;
    public static final double kRearRightOffset = 0;
  }

  public static final class ModuleConstants {
    public static final double kMaxModuleAngularSpeedRadiansPerSecond = 8 * Math.PI * 4096;
    public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 4 * Math.PI * 4096;

    public static final int kEncoderCPR = 4096; //Changed from 1024
    public static final double kWheelDiameterMeters = 0.102;
    public static final double kDriveEncoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        //(kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR
         .1403 * Math.PI * kWheelDiameterMeters / 4096.0;

    public static final double kTurningEncoderDistancePerPulse =
        // Assumes the encoders are on a 1:1 reduction with the module shaft.
        (2 * Math.PI) / (double) kEncoderCPR;

    public static final double kPModuleTurningController = 0.3;

    public static final double kPModuleDriveController = 0.1;
  }

  public static final class ArmConstants{
    public static final int kArmCANID = 9;
    public static final int kIntakeMidCANID = 10;
    public static final int kIntakeEndCANID = 11;

    public static final double kMoveArmSpeed = 0.30;

    public static final double kIntakeConeSpeed = -1.00;
    public static final double kScoreConeSpeed = 1.00;
    public static final double kIntakeCubeSpeed = 1.00;
    public static final double kScoreCubeSpeed = -1.00;
    
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kAuxControllerPort = 1;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 6;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = 4*Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = 2*Math.PI;

    public static final double kPXController = 0.5;
    public static final double kPYController = 0.5;
    public static final double kPThetaController = 0.5;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class LEDConstants{
    public static final double kOrange = 0.0;
    public static final double kYellow = 0.91;
    public static final double kPurple = 0.69;
  }
}
