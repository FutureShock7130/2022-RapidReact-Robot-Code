package frc.robot.auto.Paths;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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
import frc.robot.commands.Turret.LimelightAim;
import frc.robot.commands.Turret.TimedTurret;
import frc.robot.commands.Turret.TurretSeek;
import frc.robot.commands.Turret.TurretShoot;
import frc.robot.subsystems.Intake;

public class ThreeCargoFromOne {
    TrajectoryGenerator trajectoryGenerator;
    RobotContainer m_robot;

    String initialPathName;
    SequentialCommandGroup lastCommand;

    public ThreeCargoFromOne(RobotContainer robot) {
        m_robot = robot;
        trajectoryGenerator = new TrajectoryGenerator(m_robot.m_robotDrive);
        initialPathName = "(1) 2nd Cargo";
        lastCommand = new TwoCargoFromOne(m_robot).getCommand();
    }

    public SequentialCommandGroup getCommand() {
        return new SequentialCommandGroup(
            lastCommand,
            new AbsoluteAim(m_robot.m_robotDrive, true, -85),
            new ParallelRaceGroup(
                trajectoryGenerator.generateTranslationalPrimary(
                    "(1) 3rd Cargo Straight", 
                    new PIDController(1.3, 0.0003, 0.003),
                    new PIDController(1.4, 0.0003, 0.003)
                ),
                new SequentialCommandGroup(
                                new RunCommand(() -> {
                                        m_robot.m_robotIntake.intakeSet(0.5);
                                }, m_robot.m_robotIntake)
                        )
            ),
            new AbsoluteAim(m_robot.m_robotDrive, true, 165).withTimeout(3.0),
            trajectoryGenerator.generateTranslationalPrimary(
                "(1) 3rd Shoot Position from Straight", 
                new PIDController(1.3, 0.0003, 0.003),
                new PIDController(1.4, 0.0003, 0.003)
            ),
            new AbsoluteAim(m_robot.m_robotDrive, true, -45).withTimeout(2.0),
            new ParallelCommandGroup(
                    new TurretShoot(m_robot.m_robotTurret, 1485),
                    new SequentialCommandGroup(
                            new AbsoluteAim(m_robot.m_robotDrive, true, -45),
                            new TurretSeek(m_robot.m_robotSpinner, m_robot.m_vision),
                            new TransportUp(m_robot.m_robotIntake, m_robot.m_robotTransport),
                            new WaitCommand(0.5),
                            new TransportUp(m_robot.m_robotIntake, m_robot.m_robotTransport),
                            new WaitCommand(0.5),
                            new TransportUp(m_robot.m_robotIntake, m_robot.m_robotTransport),
                            new WaitCommand(0.5),
                            new TransportUp(m_robot.m_robotIntake, m_robot.m_robotTransport)
                    )
            ) .withTimeout(6.0)
        );
    }
}
