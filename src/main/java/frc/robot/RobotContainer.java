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
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Transporter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.MecanumControllerCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

import java.io.IOException;
import java.nio.file.Path;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems
    private final Drive m_robotDrive = new Drive();
    private final Transporter m_robotTransport = new Transporter();
    private final Turret m_robotTurret = new Turret();

    // The driver's controller
    Joystick m_driverController = new Joystick(OIConstants.kDriveTrainJoystickPort);

    // The container for the robot. Contains subsystems, OI devices, and commands.
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();

        // Configure default commands
        // Set the default drive command to split-stick arcade drive
        m_robotDrive.setDefaultCommand(
                // A split-stick arcade command, with forward/backward controlled by the left hand, and turning controlled by the right.
                new RunCommand(
                        () -> m_robotDrive.drive(
                                m_driverController.getRawAxis(OIConstants.leftStick_X),
                                -m_driverController.getRawAxis(OIConstants.leftStick_Y),
                                m_driverController.getRawAxis(OIConstants.rightStick_X),
                                false),
                        m_robotDrive));


    }

    private void configureButtonBindings() {
        // Drive at half speed when the RB button is held
        new JoystickButton(m_driverController, OIConstants.Btn_RB)
                .whenPressed(() -> m_robotDrive.setMaxOutput(DriveConstants.DriveSpeedScaler / 2))
                .whenReleased(() -> m_robotDrive.setMaxOutput(DriveConstants.DriveSpeedScaler));

        // rotate and aim at target upon pressing of the A button
        new JoystickButton(m_driverController, OIConstants.Btn_A).whenPressed(new TeleopAimCommand(m_robotDrive));

        // run the transporter subsystem upon pressing of the B button
        new JoystickButton(m_driverController, OIConstants.Btn_B).whenHeld(new TransportCommand(m_robotTransport));

        new JoystickButton(m_driverController, OIConstants.Btn_X).whenHeld(new ShootCommand(m_robotShoot));
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
            Trajectory testTrajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);

            
            // Run path following command, then stop at the end.
            return mecanumControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
        } catch (IOException e) {
            DriverStation.reportError("Unable to open JSON file", e.getStackTrace());
        }
        return null;
    }
}