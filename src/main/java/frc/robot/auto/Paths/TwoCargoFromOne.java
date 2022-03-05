package frc.robot.auto.Paths;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
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
import frc.robot.commands.Turret.TimedTurret;
import frc.robot.commands.Turret.TurretShoot;

public class TwoCargoFromOne {
    TrajectoryGenerator trajectoryGenerator;
    RobotContainer m_robot;
    String initialPathName;

    public TwoCargoFromOne(RobotContainer robot) {
        m_robot = robot;
        trajectoryGenerator = new TrajectoryGenerator(m_robot.m_robotDrive);
        initialPathName = "(1) 2nd Cargo";
    }

    public SequentialCommandGroup getCommand() {
        return new SequentialCommandGroup(
                new ParallelRaceGroup(
                        trajectoryGenerator.generateTranslationalPrimary(
                                "(1) 2nd Cargo", 
                                new PIDController(1.3, 0.0003, 0),
                                new PIDController(1.4, 0.0003, 0)
                        ),
                        new IntakeCmd(m_robot.m_robotIntake)
                ),
                trajectoryGenerator.generateTranslationalPrimary(
                        "(1) 2nd Shoot Position", 
                        new PIDController(1.3, 0.0003, 0),
                        new PIDController(1.4, 0.0003, 0)
                ),
                new AbsoluteAim(m_robot.m_robotDrive, true, 180),
                new AutoAim(m_robot.m_robotDrive, m_robot.m_robotSpinner, m_robot.m_vision).getCommand().withTimeout(4.0),
                new ParallelCommandGroup(
                        new TurretShoot(m_robot.m_robotTurret, 1500).withTimeout(6.0),
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new TransportCmd(m_robot.m_robotTransport).withTimeout(0.3),
                                        new WaitCommand(0.7),
                                        new TransportCmd(m_robot.m_robotTransport).withTimeout(0.3),
                                        new WaitCommand(0.7),
                                        new TransportCmd(m_robot.m_robotTransport).withTimeout(0.3)
                                ),
                                new IntakeCmd(m_robot.m_robotIntake).withTimeout(3.0)
                        )
                ).withTimeout(7.0)
        );         
    }
}
