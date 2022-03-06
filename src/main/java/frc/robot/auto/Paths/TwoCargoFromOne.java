package frc.robot.auto.Paths;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.auto.Actions.AutoAim;
import frc.robot.auto.Actions.TransportUp;
import frc.robot.commands.Drive.AbsoluteAim;
import frc.robot.commands.Intake.IntakeCmd;
import frc.robot.commands.Intake.TimedIntake;
import frc.robot.commands.Transporter.TimedTransport;
import frc.robot.commands.Transporter.TransportCmd;
import frc.robot.commands.Turret.LimelightAim;
import frc.robot.commands.Turret.TimedTurret;
import frc.robot.commands.Turret.TurretSeek;
import frc.robot.commands.Turret.TurretShoot;
import frc.robot.vision.Limelight;

public class TwoCargoFromOne {
    TrajectoryGenerator trajectoryGenerator;
    RobotContainer m_robot;
    String initialPathName;
    AutoAim autoAim;

    public TwoCargoFromOne(RobotContainer robot) {
        m_robot = robot;
        trajectoryGenerator = new TrajectoryGenerator(m_robot.m_robotDrive);
        initialPathName = "(1) 2nd Cargo";
        autoAim = new AutoAim(m_robot.m_robotDrive, m_robot.m_robotSpinner, m_robot.m_vision);
    }

    public SequentialCommandGroup getCommand() {
        return new SequentialCommandGroup(
                new SequentialCommandGroup(
                        new RunCommand(() -> {
                                m_robot.m_robotIntake.intakeSet(0.5);
                        }, m_robot.m_robotIntake)
                ).withTimeout(0.2),
                new ParallelRaceGroup(
                        trajectoryGenerator.generateTranslationalPrimary(
                                "(1) 2nd Cargo", 
                                new PIDController(1.3, 0.0003, 0),
                                new PIDController(1.4, 0.0003, 0)
                        ),
                        new SequentialCommandGroup(
                                new RunCommand(() -> {
                                        m_robot.m_robotIntake.intakeSet(0.5);
                                }, m_robot.m_robotIntake)
                        )
                ),
                trajectoryGenerator.generateTranslationalPrimary(
                        "(1) 2nd Shoot Position", 
                        new PIDController(1.3, 0.0003, 0),
                        new PIDController(1.4, 0.0003, 0)
                ),
                new AbsoluteAim(m_robot.m_robotDrive, true, 190).withTimeout(2.5),
                new TurretSeek(m_robot.m_robotSpinner, m_robot.m_vision).withTimeout(2.0),
                new LimelightAim(m_robot.m_vision, m_robot.m_robotSpinner),
                new ParallelDeadlineGroup(
                        new SequentialCommandGroup(
                            new WaitCommand(0.5), 
                            new TransportUp(m_robot.m_robotIntake, m_robot.m_robotTransport).withTimeout(1.5),
                            new WaitCommand(0.5),
                            new TransportUp(m_robot.m_robotIntake, m_robot.m_robotTransport).withTimeout(1.5)
                        ),
                        new TurretShoot(m_robot.m_robotTurret, 1480).withTimeout(4.0)
                )
        );         
    }
}
