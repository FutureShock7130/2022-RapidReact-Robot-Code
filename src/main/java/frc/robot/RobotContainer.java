// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.SuperstructureConstants;
import frc.robot.auto.actions.TestPathing.TestFeedforward;
import frc.robot.commands.Drive.TrapezoidProfileDrive;
import frc.robot.commands.Intake.IntakeCmd;
import frc.robot.commands.Intake.IntakeReverse;
import frc.robot.commands.Intake.IntakeStop;
import frc.robot.commands.Superstructure.AutoClimb;
import frc.robot.commands.Superstructure.SwingBack;
import frc.robot.commands.Superstructure.SwingForward;
import frc.robot.commands.Transporter.TransportCmd;
import frc.robot.commands.Transporter.TransportEject;
import frc.robot.commands.Transporter.TransportStop;
import frc.robot.commands.Turret.LimelightAim;
import frc.robot.commands.Turret.TurretSeek;
import frc.robot.commands.Turret.TurretShoot;
import frc.robot.statemachines.DriveFSM;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Spinner;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Turret;
import frc.robot.vision.Limelight;
import frc.robot.subsystems.Transporter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

import java.io.IOException;
import java.nio.file.Path;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.PathPlanner;

public class RobotContainer {
    // The robot's subsystems
    private final DriveFSM driveFSM = new DriveFSM();
    private final Drive m_robotDrive = new Drive(driveFSM);
    private final Transporter m_robotTransport = new Transporter();
    private final Turret m_robotTurret = new Turret();
    private final Intake m_robotIntake = new Intake();
    private final Limelight m_vision = new Limelight();
    private final Superstructure m_SuperStructure = new Superstructure();
    private final Spinner m_robotSpinner = new Spinner();

    // The commands
    AutoClimb autoClimb = new AutoClimb(m_SuperStructure);
    TurretShoot nearShoot = new TurretShoot(m_robotTurret, 1850);
    TurretShoot farShoot = new TurretShoot(m_robotTurret, 3000);
    SwingForward swingForward = new SwingForward(m_SuperStructure);
    SwingBack swingBack = new SwingBack(m_SuperStructure);
    IntakeCmd intake = new IntakeCmd(m_robotIntake);
    IntakeStop intakeStop = new IntakeStop(m_robotIntake);
    IntakeReverse eject = new IntakeReverse(m_robotIntake);
    TransportCmd transportCmd = new TransportCmd(m_robotTransport);
    TransportStop transportStop = new TransportStop(m_robotTransport);
    TransportEject transportEject = new TransportEject(m_robotTransport);

    BooleanSupplier targetNotIn = new BooleanSupplier() {
        @Override
        public boolean getAsBoolean() {
            if (m_vision.getV() == 0.0d)
                return true;
            return false;
        }
    };

    // The driver's controller
    Joystick m_driverController = new Joystick(OIConstants.kDriveTrainJoystickPort);
    Joystick m_operatorController = new Joystick(OIConstants.kOthersJoystickPort);

    // The container for the robot. Contains subsystems, OI devices, and commands.
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();

        m_robotDrive.setDefaultCommand(
                new RunCommand(
                        () -> {
                            m_robotDrive.drive(
                                    -m_driverController.getRawAxis(OIConstants.leftStick_Y),
                                    m_driverController.getRawAxis(OIConstants.leftStick_X),
                                    m_driverController.getRawAxis(OIConstants.rightStick_X) * 0.5,
                                    false);

                            // Superstructure Swinging
                            if (m_operatorController.getPOV() == OIConstants.POV_UP){}
                                swingForward.schedule();
                            if (m_operatorController.getPOV() == OIConstants.POV_DOWN)
                                swingBack.schedule();
                            if (m_operatorController.getPOV() == -1) {
                                swingBack.end(true);
                                swingForward.end(true);
                                swingBack.cancel();
                                swingForward.cancel();
                            }

                            // Far Flywheel Logic
                            if (m_operatorController.getRawAxis(OIConstants.trigger_R) >= 0.4) {
                                farShoot.schedule();
                            } else {
                                farShoot.cancel();
                            }

                            // Intake Logic
                            if (m_driverController.getRawAxis(OIConstants.trigger_R) >= 0.5) {
                                intake.schedule();
                            } else if (m_driverController.getRawAxis(OIConstants.trigger_L) >= 0.5) {
                                eject.schedule();
                            } else {
                                intakeStop.schedule();
                            }

                            // Transporter Logic
                            if (m_operatorController.getRawAxis(OIConstants.trigger_L) >= 0.5) {
                                transportCmd.schedule();
                            } else if (m_operatorController.getRawButton(OIConstants.Btn_LB)) {
                                transportEject.schedule();
                            } else {
                                transportStop.schedule();
                            }

                            m_SuperStructure.liftHangerRun(
                                    -m_operatorController.getRawAxis(OIConstants.leftStick_Y)
                                            * SuperstructureConstants.hangerSpeed,
                                    -m_operatorController.getRawAxis(OIConstants.rightStick_Y)
                                            * SuperstructureConstants.hangerSpeed);

                        }, m_robotDrive));
    }

    private void configureButtonBindings() {

        new JoystickButton(m_driverController, OIConstants.Btn_RB)
                .whenPressed(() -> m_robotDrive.setMaxOutput(0.5))
                .whenReleased(() -> m_robotDrive.setMaxOutput(0.95));

        new JoystickButton(m_driverController, OIConstants.Btn_Y)
                .whenHeld(new ConditionalCommand(new TurretSeek(m_robotSpinner),
                        new LimelightAim(m_robotTurret, m_vision, m_robotSpinner), targetNotIn));

        new JoystickButton(m_driverController, OIConstants.Btn_B)
                .whileHeld(new RunCommand(() -> {
                    m_robotSpinner.spinnerRun(0.3);
                }, m_robotSpinner))
                .whenReleased(new RunCommand(() -> {
                    m_robotSpinner.spinnerRun(0.0);
                }, m_robotSpinner));

        new JoystickButton(m_driverController, OIConstants.Btn_X)
                .whileHeld(new RunCommand(() -> {
                    m_robotSpinner.spinnerRun(-0.3);
                }, m_robotSpinner))
                .whenReleased(new RunCommand(() -> {
                    m_robotSpinner.spinnerRun(0.0);
                }, m_robotSpinner));

        new JoystickButton(m_operatorController, OIConstants.Btn_RB)
                .whenHeld(nearShoot)
                .whenReleased(() -> nearShoot.cancel());

        // new JoystickButton(m_operatorController,
        // OIConstants.Btn_RB).whenPressed(autoClimb);
        // new JoystickButton(m_operatorController, OIConstants.Btn_LB).whenPressed(()
        // -> autoClimb.cancel());
    }

    // Use this to pass the autonomous command to the main {@link Robot} class.
    public void testDrive() {
        m_robotDrive.testMotor();
    }

    public Pose2d getPose() {
        return m_robotDrive.getPose();
    }

    public Command getAutonomousCommand() {
        TestFeedforward m_command1 = new TestFeedforward(m_robotDrive);
        TrapezoidProfileDrive m_command2 = new TrapezoidProfileDrive(4.0, m_robotDrive);
        // try {

        // // Run path following command, then stop at the end.
        // // return mecanumControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0,
        // // false));
        // } catch (IOException e) {
        // DriverStation.reportError("Unable to open JSON file", e.getStackTrace());
        // }
        return m_command1;
    }
}
