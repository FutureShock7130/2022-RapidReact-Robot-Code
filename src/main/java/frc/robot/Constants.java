// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

// The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. 
// This class should not be used for any other purpose. 
// All constants should be declared globally (i.e. public static). Do not put anything functional in this class.

public final class Constants {
  public static final class DriveConstants {
    public static final int kFrontLeftMotorID = 1;
    public static final int kRearLeftMotorID = 2;
    public static final int kFrontRightMotorID = 3;
    public static final int kRearRightMotorID = 4;

    public static final double kTrackWidth = 0.5682;
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = 0.4904;
    // Distance between centers of front and back wheels on robot

    public static final double DriveSpeedScaler = 0.7;

    public static final DifferentialDriveKinematics kDifferentialDriveKinematics = new DifferentialDriveKinematics(kTrackWidth);

    public static final MecanumDriveKinematics kMecanumDriveKinematics = new MecanumDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    public static final int kEncoderCPR = 2048;
    public static final double kWheelDiameterMeters = 0.1524;
    public static final double kWheelCircumference = kWheelDiameterMeters * Math.PI;
    public static final double kGearRatio = 12.5;
    public static final double kEncoderDistancePerPulse = kWheelCircumference / (double) kEncoderCPR / kGearRatio
        / Math.sqrt(2) * 10;
    public static final double kPulseToDistance = kWheelCircumference / (double) kEncoderCPR / kGearRatio
    / Math.sqrt(2);
      
    public static final int kPIDfCPR = 1023;

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or
    // theoretically for "your" robot's drive.
    // The SysId tool provides a convenient method for obtaining these values for
    // your robot.

    public static final double kS = 0.65204; //0.76364
    public static final double kV = 3.99;
    public static final double kA = 0.32395; //0.22548
    public static final double kJ = 14;

    public static final double kPDriveVel = 4.7078; //0.3 //3.7308
    public static final double kDDriveVel = 0.012;

    public static final SimpleMotorFeedforward kDriveFeedforward = new SimpleMotorFeedforward(0, 0, 0);

    public static final SimpleMotorFeedforward kFeedforward = new SimpleMotorFeedforward(kS, kV, kA);

    public static PIDController velController = new PIDController(kPDriveVel, 0, 0);

    public static PIDController idealXController = new PIDController(0.4, 0.00003, 0.00006);
    public static PIDController idealYController = new PIDController(0.4, 0.00006, 0.00006);

    public static ProfiledPIDController idealThetaController = new ProfiledPIDController(
      0.003, 0.0, 0.0,
      new TrapezoidProfile.Constraints(Math.PI * 1.4, Math.PI / 2)
    );


    public static final double kSlewRate = 0.95;

    // Ramsete Parameters
    public static final double kRamseteB = 2.0;
    public static final double kRamseteTheta = 0.7;

    public static final double kMaxVelocityMetersPerSecond = 3.0;
    public static final double kMaxAccelerationMetersPerSecondSquared = 2.0;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

    // Example value only - as above, this must be tuned for your drive!
    public static final double kPFrontLeftVel = 0.3;
    public static final double kPRearLeftVel = 0.3;
    public static final double kPFrontRightVel = 0.3;
    public static final double kPRearRightVel = 0.3;
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
    public static final int POV_UP = 0;
    public static final int POV_DOWN = 180;
    public static final int POV_RIGHT = 90;
    public static final int POV_LEFT = 270;
  }

  public static final class TurretConstants {
    public static final int kMasterFlyWheelID = 12;
    public static final int kSlaveFlyWheelID = 13;

    // Spinner Related Constants and Switch OIs
    public static final int forwardLimitSwitch = 1;
    public static final int reverseLimitSwitch = 0;
    public static final double turretSpeedMultiplier = 0.3;
    public static final int kSpinnerID = 11;

    // Flywheel Feedforward Constants
    public static final double kS = 0.0118;
    public static final double kV = 0.00002;
    public static final double kA = 0.00012;

    public static final double kRPMtoLinearVelocity = 1 / 60 * 0.4788;

    public static final double closeVoltage = 0.32;
    public static final double farVoltage = 0.45;
  }

  public static final class IntakeConstants {
    public static final int kIntakeID = 10;
    public static final int kDownTransporterID = 5;
    public static final double intakeSpeed = 0.9;
    public static final double transportSpeed = 0.75;
    public static final double idealIntakeDt = 0.75;
  }

  public static final class TransporterConstants {
    public static final int kTopTransporterID = 9;
    public static final int kDownTransporterID = 5;
    public static final double transportSpeed = 0.75;
    public static final double transportTime = 1;

    public static final double idealTransportDt = 0.75;
  }

  public static final class SuperstructureConstants {
    public static final int kSwingLeftID = 6;
    public static final int kSwingRightID = 8;
    public static final int kHangerLeftID = 14;
    public static final int kHangerRightID = 15;
    public static final int LlimitSwitch = 2;
    public static final int RlimitSwitch = 3;

    public static final double swingSpeed = 0.3;
    public static final double hangerSpeed = 0.98;
    public static final double HangerMaxPosition = -54; // Need to be tested, in rotation unit
    public static final double HangerMinPosition = -176;
    public static final double SwingMaxPosition = 5000; // Need to be tested, in rotation unit
    public static final double behindGrabEncoderUnit = Math.atan(0.405 / 0.6) / 360
        * SuperstructureConstants.TalonSRXCPR * SuperstructureConstants.SwingGearRatio;
    public static final double SwingGearRatio = 205.71;
    public static final double TalonSRXCPR = 4096;
    public static final double NeoEncoderCPR = 42;
  }

  public static final class AutoConstants {
    public static final double kPXController = 1.356726;
    public static final double kPYController = 1.356726;
    public static final double kDXYController = 0;
    public static final double kPThetaController = 0.5;

    // Constraint for the motion profilied robot angle controller
    
  }

  public static final class LimelightConstants {
    // How many degrees back is your limelight rotateed from perfectly vertical
    // Check our own limelight degree
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
}
