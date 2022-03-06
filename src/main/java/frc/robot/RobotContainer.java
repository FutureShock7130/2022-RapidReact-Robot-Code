// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.SuperstructureConstants;
import frc.robot.Constants.TransporterConstants;
import frc.robot.auto.Actions.TestPathing.TestFeedforward;
import frc.robot.commands.Drive.TrapezoidProfileDrive;
import frc.robot.commands.Intake.IntakeCmd;
import frc.robot.commands.Intake.IntakeReverse;
import frc.robot.commands.Intake.IntakeStop;
import frc.robot.commands.Superstructure.AutoClimb;
import frc.robot.commands.Superstructure.SwingBack;
import frc.robot.commands.Superstructure.SwingForward;
import frc.robot.commands.Transporter.TimedTransport;
import frc.robot.commands.Transporter.TransportCmd;
import frc.robot.commands.Transporter.TransportEject;
import frc.robot.commands.Transporter.TransportStop;
import frc.robot.commands.Turret.LimelightAim;
import frc.robot.commands.Turret.PassiveFlywheel;
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
import edu.wpi.first.wpilibj2.command.MecanumControllerCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

import java.util.function.BooleanSupplier;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

public class RobotContainer {
    // The robot's subsystems
    public final DriveFSM driveFSM = new DriveFSM();
    public final Drive m_robotDrive = new Drive(driveFSM);
    public final Transporter m_robotTransport = new Transporter();
    public final Turret m_robotTurret = new Turret();
    public final Intake m_robotIntake = new Intake();
    public final Limelight m_vision = new Limelight();
    public final Superstructure m_SuperStructure = new Superstructure();
    public final Spinner m_robotSpinner = new Spinner();

    // The commands
    SwingForward swingForward = new SwingForward(m_SuperStructure);
    SwingBack swingBack = new SwingBack(m_SuperStructure);
    IntakeCmd intake = new IntakeCmd(m_robotIntake);
    IntakeStop intakeStop = new IntakeStop(m_robotIntake);
    IntakeReverse eject = new IntakeReverse(m_robotIntake);
    TransportCmd transportCmd = new TransportCmd(m_robotTransport);
    TransportStop transportStop = new TransportStop(m_robotTransport);
    TransportEject transportEject = new TransportEject(m_robotTransport);
    PassiveFlywheel shootPassiveState = new PassiveFlywheel(m_robotTurret);
    AutoClimb autoClimb = new AutoClimb(m_SuperStructure);
    TurretShoot nearShoot = new TurretShoot(m_robotTurret, 1750);
 

    private final SimpleMotorFeedforward feedforward = DriveConstants.kFeedforward;

    BooleanSupplier targetStatus = new BooleanSupplier() {
        @Override
        public boolean getAsBoolean() {
            if (m_vision.getTargetStatus()) {
                return true;
            }
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
                                    m_driverController.getRawAxis(OIConstants.rightStick_X) * 0.7,
                                    false);
    
                            // Superstructure Swinging
                            if (m_operatorController.getPOV() == OIConstants.POV_UP) {
                                swingForward.schedule();
                            }
                            if (m_operatorController.getPOV() == OIConstants.POV_DOWN) {
                                swingBack.schedule();
                            }

                            if (m_operatorController.getPOV() == OIConstants.POV_LEFT) {
                                swingBack.end(true);
                                swingForward.end(true);
                                swingBack.cancel();
                                swingForward.cancel();
                            }

                            // Superstructure Lifting Logic
                            m_SuperStructure.liftHangerRun(
                                    -m_operatorController.getRawAxis(OIConstants.leftStick_Y)
                                            * SuperstructureConstants.hangerSpeed,
                                    -m_operatorController.getRawAxis(OIConstants.rightStick_Y)
                                            * SuperstructureConstants.hangerSpeed);
                        }, m_robotDrive));

        // m_robotIntake.setDefaultCommand(
        //         new RunCommand(() -> {
        //             // Intake Logic
        //             if (m_driverController.getRawAxis(OIConstants.trigger_R) >= 0.5) {
        //                 new RunCommand(() -> m_robotIntake.intakeRun(), m_robotIntake);
        //             } else if (m_driverController.getRawAxis(OIConstants.trigger_L) >= 0.5) {
        //                 new RunCommand(() -> m_robotIntake.intakeReverse(), m_robotIntake);
    
        //             } else {
        //                 new RunCommand(() -> m_robotIntake.intakeStop(), m_robotIntake);
                        
        //             }
        //         }, m_robotIntake));

        // m_robotTransport.setDefaultCommand(
        //         new RunCommand(() -> {
        //             // Transporter Logic
        //             if (m_operatorController.getRawAxis(OIConstants.trigger_L) >= 0.5) {
        //                 transportCmd.schedule();
        //             } else if (m_operatorController.getRawAxis(OIConstants.trigger_R) >= 0.5) {
        //                 transportEject.schedule();
        //             } else {
        //                 transportStop.schedule();
        //             }
        //         }, m_robotTransport));
        m_robotIntake.setDefaultCommand(
                new RunCommand(() -> {
                    // Intake Logic
                    m_robotIntake.intakeSet(
                        m_driverController.getRawAxis(OIConstants.trigger_R) - m_driverController.getRawAxis(OIConstants.trigger_L)
                    );
                }, m_robotIntake));

        // m_robotTransport.setDefaultCommand(
        //         new RunCommand(() -> {
        //             // Transporter Logic
        //             if (m_operatorController.getRawAxis(OIConstants.trigger_R) > 0.1) {
        //                 m_robotTransport.transportRun(TransporterConstants.transportSpeed);
        //             } else {
        //                 m_robotTransport.transportRun(0);
        //             }
        //         }, m_robotTransport));

        // m_SuperStructure.setDefaultCommand(
        //         new RunCommand(() -> {
        //             // Superstructure Lifting Logic
        //             m_SuperStructure.liftHangerRun(
        //                     -m_operatorController.getRawAxis(OIConstants.leftStick_Y)
        //                             * SuperstructureConstants.hangerSpeed,
        //                     -m_operatorController.getRawAxis(OIConstants.rightStick_Y)
        //                             * SuperstructureConstants.hangerSpeed);

        //             // Superstructure Swinging Forward
        //             if (m_operatorController.getPOV() == OIConstants.POV_UP) {
        //                 swingForward.schedule();
        //             }

        //             // Superstucture Swinging Backward
        //             if (m_operatorController.getPOV() == OIConstants.POV_DOWN) {
        //                 swingBack.schedule();
        //             }

        //             // Superstructure Stop
        //             if(m_operatorController.getPOV()==-1){
        //                 swingBack.end(true);
        //                 swingForward.end(true);
        //                 swingBack.cancel();
        //                 swingForward.cancel();
        //             }
        //         }, m_SuperStructure));
    }

    private void configureButtonBindings() {

        // drivetrain sub
        new JoystickButton(m_driverController, OIConstants.Btn_RB)
                .whenPressed(() -> m_robotDrive.setMaxOutput(0.5))
                .whenReleased(() -> m_robotDrive.setMaxOutput(0.95));


        // spinner auto aim
        new JoystickButton(m_operatorController, OIConstants.Btn_Y)
                .whenHeld(new TurretSeek(m_robotSpinner, m_vision))
                .whenReleased(
                        new LimelightAim(m_vision, m_robotSpinner));

        // spinner spin right
        new JoystickButton(m_operatorController, OIConstants.Btn_B)
                .whenHeld(new RunCommand(() -> {
                    m_robotTransport.transportRun(0.75);
                }, m_robotTransport))
                .whenReleased(new RunCommand(() -> {
                    m_robotTransport.transportRun(0.0);
                }, m_robotTransport));

        // spinner spin left
        new JoystickButton(m_operatorController, OIConstants.Btn_X)
                .whenHeld(new RunCommand(() -> {
                    m_robotTransport.transportRun(-0.75);
                }, m_robotTransport))
                .whenReleased(new RunCommand(() -> {
                    m_robotTransport.transportRun(0.0);
                }, m_robotTransport));

        // For testing
        // Timed Transportation
        new JoystickButton(m_driverController, OIConstants.Btn_A)
                .whenPressed(
                        new TimedTransport(0.4, TransporterConstants.idealTransportDt, m_robotTransport));

        // Shooting Bindings
        new JoystickButton(m_operatorController, OIConstants.Btn_A)
                .whenHeld(new TurretShoot(m_robotTurret, 1550));

        
        new JoystickButton(m_operatorController, OIConstants.Btn_RB)
                .whenHeld(new TurretShoot(m_robotTurret, 1750));

    

        // new JoystickButton(m_operatorController, OIConstants.Btn_LB)
                // .whenHeld(farShoot);

        // new JoystickButton(m_operatorController,
        // OIConstants.Btn_RB).whenPressed(autoClimb);
        // new JoystickButton(m_operatorController, OIConstants.Btn_LB).whenPressed(()
        // -> autoClimb.cancel());
    }

    public Pose2d getPose() {
        return m_robotDrive.getPose();
    }

    // AUTONOMOUS COMMANDS

    public Command getAutonomousCommand() {
        TestFeedforward m_command = new TestFeedforward(m_robotDrive);
        TrapezoidProfileDrive m_command2 = new TrapezoidProfileDrive(4.0, m_robotDrive);
        return m_command;
    }

    // Use this to pass the autonomous command to the main {@link Robot} class.
    // @return the command to run in autonomous

    public Command getAutonomousTrajectoryCommand() {
        driveFSM.setOdometryMecanum();

        PathPlannerTrajectory trajectoryPathPlanner = PathPlanner.loadPath("New New Path",
                DriveConstants.kMaxVelocityMetersPerSecond, DriveConstants.kMaxAccelerationMetersPerSecondSquared);

        PathPlannerState state = (PathPlannerState) trajectoryPathPlanner.getEndState();
        System.out.println(state);

        MecanumControllerCommand betterMecanumCommand = new MecanumControllerCommand(
                trajectoryPathPlanner,
                m_robotDrive::getMecanumPose,
                DriveConstants.kFeedforward,
                DriveConstants.kMecanumDriveKinematics,
                DriveConstants.idealXController,
                DriveConstants.idealYController,
                DriveConstants.idealThetaController,
                DriveConstants.kMaxVelocityMetersPerSecond,
                new PIDController(DriveConstants.kPDriveVel, 0, 0),
                new PIDController(DriveConstants.kPDriveVel, 0, 0),
                new PIDController(DriveConstants.kPDriveVel, 0, 0),
                new PIDController(DriveConstants.kPDriveVel, 0, 0),
                m_robotDrive::getCurrentMecanumWheelSpeeds,
                m_robotDrive::setDriveMotorControllersVolts,
                m_robotDrive);

        // Reset odometry to the starting pose of the trajectory.
        // m_robotDrive.resetGyro();
        m_robotDrive.resetOdometry(trajectoryPathPlanner.getInitialPose());
        // Run path following command, then stop at the end.
        return betterMecanumCommand.andThen(() -> m_robotDrive.differentialDriveVolts(0, 0));
    }
}
