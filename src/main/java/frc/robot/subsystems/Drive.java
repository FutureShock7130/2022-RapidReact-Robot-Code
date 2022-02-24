// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.MecanumDriveMotorVoltages;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

public class Drive extends SubsystemBase {

  private final WPI_TalonFX motorFL = new WPI_TalonFX(DriveConstants.kFrontLeftMotorPort);
  private final WPI_TalonFX motorRL = new WPI_TalonFX(DriveConstants.kRearLeftMotorPort);
  private final WPI_TalonFX motorFR = new WPI_TalonFX(DriveConstants.kFrontRightMotorPort);
  private final WPI_TalonFX motorRR = new WPI_TalonFX(DriveConstants.kRearRightMotorPort);

  private final MecanumDrive m_drive = new MecanumDrive(motorFL, motorRL, motorFR, motorRR);

  // The gyro sensor
  private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);

  // Odometry class for tracking robot pose
  MecanumDriveOdometry m_odometry = new MecanumDriveOdometry(DriveConstants.kDriveKinematics, m_gyro.getRotation2d());

  /** Creates a new DriveSubsystem. */
  public Drive() {
    // Sets the distance per pulse for the encoders
    // DO WE NEED TO DO THIS?

    // We need to invert one side of the drivetrain so that positive voltages result in both sides moving forward. 
    // Depending on how your robot's gearbox is constructed, you might have to invert the left side instead.
    motorFR.setInverted(true);
    motorRR.setInverted(true);
    motorFL.setInverted(false);
    motorRL.setInverted(false);

    resetEncoders();
    resetOdometry(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        m_gyro.getRotation2d(),
        new MecanumDriveWheelSpeeds(
          motorFL.getSelectedSensorVelocity() * DriveConstants.kEncoderDistancePerPulse,
          motorFR.getSelectedSensorVelocity() * DriveConstants.kEncoderDistancePerPulse,
          motorRL.getSelectedSensorVelocity() * DriveConstants.kEncoderDistancePerPulse,
          motorRR.getSelectedSensorVelocity() * DriveConstants.kEncoderDistancePerPulse));

    SmartDashboard.putNumber("velocity", motorFL.getSelectedSensorVelocity() * DriveConstants.kEncoderDistancePerPulse);
    SmartDashboard.putNumber("Pose X", m_odometry.getPoseMeters().getX());
    SmartDashboard.putNumber("Pose Y", m_odometry.getPoseMeters().getY());

  }

  // Returns the currently-estimated pose of the robot.
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  // Resets the odometry to the specified pose.
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(pose, m_gyro.getRotation2d());
  }

  // Drives the robot at given x, y and theta speeds. 
  // Speeds range from [-1, 1] and the linear speeds have no effect on the angular speed.
  // @param xSpeed - Speed of the robot in the x direction (forward/backwards).
  // @param ySpeed - Speed of the robot in the y direction (sideways).
  // @param rot    - Angular rate of the robot.
  // @param fieldRelative Whether the provided x and y speeds are relative to the field.

  @SuppressWarnings("ParameterName")
  public void drive(double ySpeed, double xSpeed, double rot) {
      m_drive.driveCartesian(ySpeed, xSpeed, rot);
  }

  // Drives the robot at given x, y and theta speeds. 
  // Speeds range from [-1, 1] and the linear speeds have no effect on the angular speed.
  // @param xSpeed - Speed of the robot in the x direction (forward/backwards).
  // @param ySpeed - Speed of the robot in the y direction (sideways).
  // @param rot    - Angular rate of the robot.
  // @param fieldRelative Whether the provided x and y speeds are relative to the field.

  @SuppressWarnings("ParameterName")
  public void drive(double ySpeed, double xSpeed, double rot, boolean fieldRelative) {
    if (fieldRelative) {
      m_drive.driveCartesian(ySpeed, xSpeed, rot, -m_gyro.getAngle());
    } else {
      m_drive.driveCartesian(ySpeed, xSpeed, rot);
    }
  }

  /** Sets the front left drive MotorController to a voltage. */
  public void setDriveMotorControllersVolts(MecanumDriveMotorVoltages volts) {
    motorFL.setVoltage(volts.frontLeftVoltage);
    motorRL.setVoltage(volts.rearLeftVoltage);
    motorFR.setVoltage(volts.frontRightVoltage);
    motorRR.setVoltage(volts.rearRightVoltage);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    motorFL.setSelectedSensorPosition(0);
    motorRL.setSelectedSensorPosition(0);
    motorFR.setSelectedSensorPosition(0);
    motorRR.setSelectedSensorPosition(0);
  }

  // Gets the current wheel speeds.
  public MecanumDriveWheelSpeeds getCurrentWheelSpeeds() {
    return new MecanumDriveWheelSpeeds(
        motorFL.getSelectedSensorVelocity() * DriveConstants.kEncoderDistancePerPulse,
        motorFR.getSelectedSensorVelocity() * DriveConstants.kEncoderDistancePerPulse,
        motorRL.getSelectedSensorVelocity() * DriveConstants.kEncoderDistancePerPulse,
        motorRR.getSelectedSensorVelocity() * DriveConstants.kEncoderDistancePerPulse);
  }

  // Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  // Returns the heading of the robot. (from -180 to 180)
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  // the turn rate of the robot.(in degrees per second)
  public double getTurnRate() {
    return -m_gyro.getRate();
  }

  public void testMotor() {
    while ((motorFR.getSelectedSensorPosition() / 2048 * DriveConstants.kWheelCircumference / DriveConstants.kGearRatio) < 2) {
      m_drive.driveCartesian(0, -0.1, 0);
    }
    m_drive.driveCartesian(0, 0, 0);
  }
}
