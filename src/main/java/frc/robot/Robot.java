// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.auto.AutoModePlanner;
import frc.robot.auto.AutoModes;
import frc.robot.auto.Actions.TestPathing.TestFeedforward;
import frc.robot.auto.Paths.OneCargo;
import frc.robot.auto.Paths.ThreeCargoFromOne;
import frc.robot.auto.Paths.ThreeCargoFromTwo;
import frc.robot.auto.Paths.TurretTest;
import frc.robot.auto.Paths.TwoCargoFromOne;
import frc.robot.auto.Paths.TwoCargoFromThree;
import frc.robot.commands.Reset.ResetZero;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private Telemetry telemetry;

  Joystick m_driverController;
  Joystick m_operatorController;

  SendableChooser<SequentialCommandGroup> m_autoChooser = new SendableChooser<>();
  AutoModePlanner autoPlanner = new AutoModePlanner(m_robotContainer);

  SequentialCommandGroup m_auto1;
  SequentialCommandGroup m_auto21;
  SequentialCommandGroup m_auto31;
  SequentialCommandGroup m_auto22;
  SequentialCommandGroup m_auto32;
  SequentialCommandGroup m_auto23;
  SequentialCommandGroup m_autoTest;
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    UsbCamera camera = CameraServer.startAutomaticCapture();
    CvSink cvSink = CameraServer.getVideo();
    CvSource outputStream = CameraServer.putVideo("Blur", 1920, 1080);
    telemetry = new Telemetry();

    Joystick m_driverController = new Joystick(OIConstants.kDriveTrainJoystickPort);
    Joystick m_operatorController = new Joystick(OIConstants.kOthersJoystickPort);

    m_auto1 = new OneCargo(m_robotContainer).getCommand();
    m_auto21 = new TwoCargoFromOne(m_robotContainer).getCommand();
    m_auto31 = new ThreeCargoFromOne(m_robotContainer).getCommand();
    m_auto32 = new ThreeCargoFromTwo(m_robotContainer).getCommand();
    m_auto23 = new TwoCargoFromThree(m_robotContainer).getCommand();
    m_autoTest = new TurretTest(m_robotContainer).getCommand();

    // m_autoChooser.setDefaultOption("Default", autoPlanner.handleAutoMode(AutoModes.StartingPosition.TWO, AutoModes.DriveStrategy.TWO_CARGO));
    // m_autoChooser.addOption("Two Cargo from 2", autoPlanner.handleAutoMode(AutoModes.StartingPosition.TWO, AutoModes.DriveStrategy.TWO_CARGO));
    // m_autoChooser.addOption("Two Cargo from 3", autoPlanner.handleAutoMode(AutoModes.StartingPosition.THREE, AutoModes.DriveStrategy.TWO_CARGO));
    // m_autoChooser.addOption("Three Cargo from 1", autoPlanner.handleAutoMode(AutoModes.StartingPosition.ONE, AutoModes.DriveStrategy.THREE_CARGO));
    // m_autoChooser.addOption("Three Cargo from 2", autoPlanner.handleAutoMode(AutoModes.StartingPosition.TWO, AutoModes.DriveStrategy.THREE_CARGO));
    // m_autoChooser.addOption("Three Cargo from 3", autoPlanner.handleAutoMode(AutoModes.StartingPosition.THREE, AutoModes.DriveStrategy.THREE_CARGO));
    // m_autoChooser.addOption("Four Cargo from 1", autoPlanner.handleAutoMode(AutoModes.StartingPosition.ONE, AutoModes.DriveStrategy.FOUR_CARGO));
    // m_autoChooser.addOption("Four Cargo from 3 B", autoPlanner.handleAutoMode(AutoModes.StartingPosition.THREE, AutoModes.DriveStrategy.FOUR_CARGO));

    SmartDashboard.putData(m_autoChooser);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  // The Following are autonomous commands that can be accessed with the Shuffleboard or Glass GUI
  

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    ResetZero reset = new ResetZero(m_robotContainer);
    reset.schedule();

    // m_autonomousCommand = autoPlanner.handleAutoMode();
    // m_autonomousCommand = m_autoChooser.getSelected();

    PathPlannerTrajectory trajectory = PathPlanner.loadPath("(1) 2nd Cargo", DriveConstants.kMaxVelocityMetersPerSecond, DriveConstants.kMaxAccelerationMetersPerSecondSquared);
    m_robotContainer.m_robotDrive.resetOdometry(trajectory.getInitialPose());
    
    // schedule the autonomous command (example)
    m_autoTest.schedule();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_auto1 != null) {
      m_auto1.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }
}
