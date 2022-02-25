// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.Intake.IntakeCmd;
import frc.robot.commands.Intake.IntakeReverse;
import frc.robot.commands.Intake.IntakeStop;
import frc.robot.commands.Turret.AbsoluteAim;
import frc.robot.commands.Turret.LimelightAim;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Turret;
import frc.robot.vision.Limelight;
import frc.robot.subsystems.Transporter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.MecanumControllerCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

import java.io.IOException;
import java.nio.file.Path;

public class RobotContainer {
    // The robot's subsystems
    private final Drive m_robotDrive = new Drive();
    private final Transporter m_robotTransport = new Transporter();
    private final Turret m_robotTurret = new Turret();
    private final Intake m_robotIntake = new Intake();

    private final Limelight m_vision = new Limelight();

    // The driver's controller
    Joystick m_driverController = new Joystick(OIConstants.kDriveTrainJoystickPort);
    Joystick m_operatorController = new Joystick(OIConstants.kOthersJoystickPort);

    // The container for the robot. Contains subsystems, OI devices, and commands.
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();

        m_robotDrive.setDefaultCommand(
                new RunCommand(
                        () -> m_robotDrive.drive(
                                -m_driverController.getRawAxis(OIConstants.leftStick_Y),
                                m_driverController.getRawAxis(OIConstants.leftStick_X),
                                -m_driverController.getRawAxis(OIConstants.rightStick_X),
                                false),
                        m_robotDrive));

    }

    private void configureButtonBindings() {
        // Drive at half speed when the RB button is held
        new JoystickButton(m_driverController, OIConstants.Btn_RB)
                .whileHeld(() -> m_robotDrive.setMaxOutput(DriveConstants.DriveSpeedScaler * 0.7))
                .whenReleased(() -> m_robotDrive.setMaxOutput(DriveConstants.DriveSpeedScaler));

        new JoystickButton(m_driverController, OIConstants.Btn_LB)
                .whileHeld(() -> m_robotDrive.setMaxOutput(DriveConstants.DriveSpeedScaler * 0.4))
                .whenReleased(() -> m_robotDrive.setMaxOutput(DriveConstants.DriveSpeedScaler));

        new JoystickButton(m_driverController, OIConstants.trigger_L)
        .whileHeld(new IntakeCmd(m_robotIntake))
        .whenReleased(new IntakeStop(m_robotIntake));

        new JoystickButton(m_driverController, OIConstants.trigger_R)
        .whileHeld(new IntakeReverse(m_robotIntake))
        .whenReleased(new IntakeStop(m_robotIntake));
        ;

        new JoystickButton(m_operatorController, OIConstants.Btn_X)
        .whenHeld(new LimelightAim(m_robotTurret, m_vision))
        ;

        new JoystickButton(m_operatorController, OIConstants.Btn_B)
        .whileHeld(new RunCommand(() -> {
            m_robotTurret.spinnerRun(0.3);
        }, m_robotTurret))
        .whenReleased(new RunCommand(() -> {
            m_robotTurret.spinnerRun(0.0);
        }, m_robotTurret));
        new JoystickButton(m_operatorController, OIConstants.Btn_A)
        .whileHeld(new RunCommand(() -> {
            m_robotTurret.spinnerRun(-0.3);
        }, m_robotTurret))
        .whenReleased(new RunCommand(() -> {
            m_robotTurret.spinnerRun(0.0);
        }, m_robotTurret));

    }

    // Use this to pass the autonomous command to the main {@link Robot} class.
    public void testDrive() {
        m_robotDrive.testMotor();
    }

    public Pose2d getPose() {
        return m_robotDrive.getPose();
    }   

    public Command getAutonomousCommand() {

        String trajectoryJSON = "paths/straightTest.wpilib.json";
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
            Trajectory Trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);

            


            // Run path following command, then stop at the end.
            // return mecanumControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
        } catch (IOException e) {
            DriverStation.reportError("Unable to open JSON file", e.getStackTrace());
        }
        return null;
    }
}
