// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;

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
    public static final int kLeftMotor1Port = 3;
    public static final int kLeftMotor2Port = 4;
    public static final int kRightMotor1Port = 1;
    public static final int kRightMotor2Port = 2;

    public static final double kTrackwidthMeters = 0.4482;
    public static final MecanumDriveKinematics kDriveKinematics =
        new MecanumDriveKinematics();

    public static final int kEncoderCPR = 4096;
    public static final double kWheelDiameterMeters = 0.1016;
    public static final double kWheelCircumference = kWheelDiameterMeters * Math.PI;
    public static final double kGearRatio = 1;//(50 / 14) * (48 / 16);
    public static final double kEncoderDistancePerPulse = kWheelCircumference / (double) kEncoderCPR / kGearRatio * 10;
    public static final double kEncoderDistanceRatio = kWheelCircumference / (double) kEncoderCPR / kGearRatio;

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or theoretically
    // for *your* robot's drive.
    // The Robot Characterization Toolsuite provides a convenient tool for obtaining these
    // values for your robot.
    public static final double ksVolts = 0.55652;
    public static final double kvVoltSecondsPerMeter = 0.020654;
    public static final double kaVoltSecondsSquaredPerMeter = 0.001253;

    public static final SimpleMotorFeedforward feedForward = 
        new SimpleMotorFeedforward(ksVolts, kvVoltSecondsPerMeter, kaVoltSecondsSquaredPerMeter);

    // Example value only - as above, this must be tuned for your drive!
    public static final double kPDriveVel = 0.57006;
    public static final double kDDriveVel = 0.060167;
  }

  public static final class OIConstants {
    public static final int kDriveTrainJoystickPort = 0;
    public static final int kOthersJoystickPort = 1;

    public static final int leftStick_X = 0;
    public static final int leftStick_Y = 1;
    public static final int rightStick_X = 4;
    public static final int rightStick_Y = 5;
    public static final int trigger_L = 2;
    public static final int trigger_R = 3;
    public static final int Btn_A = 1;
    public static final int Btn_B = 2;
    public static final int Btn_X = 3;
    public static final int Btn_Y = 4;
    public static final int Btn_LB = 5;
    public static final int Btn_RB = 6;
    public static final int Btn_LS = 9;
    public static final int Btn_RS = 10;
  }

  public static final class TurrentConstants {
    public static final int kMasterFlyWheelID = 12;
    public static final int kSlaveFlyWheelID = 13;
    public static final int kSpinnerID = 11;
    public static final int forwardLimitSwitch = 1;
    public static final int reverseLimitSwitch = 0;
  }

  public static final class IntakeConstants {
    public static final int kintakeID = 10;
    public static final double intakeSpeed = 0.7;
  }

  public static final class TransporterConstants {
    public static final int kTopTransporterID = 9;
    public static final int kDownTransporterID = 5;
    public static final double transportSpeed = 0.0;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1;

    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
  }

  public static final class LimelightConstants {
        // how many degrees back is your limelight rotateed from perfectly vertical
    // check our own limelight degree
    public static final double limelightMounAngleDegrees = 15.0;

    // distance from the center of the limelight lens to the floor
    public static final double limelightLensHeightMeters = 0.8;

    // distance from the target to the floor
    public static final double goalHeightMeters = 2.6;

    // define constants
    public static final double KpDistance = -0.1f;
    public static final double KpAim = 0;
    public static final double min_command = 0;
    public static final double current_distance = 0; // distanceFromLimeligtToGoalInches
  }

  public static final class SuperstructureConstants {
    public static final int kSwingLeftID = 7;
    public static final int kSwingRightID = 8;
    public static final int kHangerLeftID = 14;
    public static final int kHangerRightID = 15;
    public static final double swingSpeed = 0.3;
    public static final double hangerSpeed = 0.3;
  }
}