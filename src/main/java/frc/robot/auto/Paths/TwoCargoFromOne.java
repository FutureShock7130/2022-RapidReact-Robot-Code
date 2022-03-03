package frc.robot.auto.Paths;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.auto.Actions.AutoAim;
import frc.robot.commands.Intake.TimedIntake;
import frc.robot.commands.Transporter.TimedTransport;
import frc.robot.commands.Turret.TimedTurret;

public class TwoCargoFromOne {
    TrajectoryGenerator trajectoryGenerator;
    RobotContainer m_robot;

    public TwoCargoFromOne(RobotContainer robot) {
        m_robot = robot;
        trajectoryGenerator = new TrajectoryGenerator(m_robot.m_robotDrive);
    }

    public SequentialCommandGroup getCommand() {
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        trajectoryGenerator.generateTranslationalPrimary(
                                "1 to 2 from 1",
                                new PIDController(0.4, 0.003, 0.003), 
                                new PIDController(0.4, 0.003, 0.003)
                            ),
                        new TimedIntake(2.0, m_robot.m_robotIntake)
                ),
                new ParallelCommandGroup(
                        trajectoryGenerator.generate(
                            "2 to 2s from 1",
                            new PIDController(0.13, 0.003, 0.003),
                            new PIDController(0.13, 0.003, 0.003),
                            new ProfiledPIDController(
                                1.3, 0.01, 0.013, 
                                new TrapezoidProfile.Constraints(DriveConstants.kMaxVelocityMetersPerSecond, DriveConstants.kMaxAccelerationMetersPerSecondSquared))
                        ),
                        new SequentialCommandGroup(
                                new TimedIntake(0.5, m_robot.m_robotIntake),
                                new WaitCommand(0.5),
                                new TimedTransport(1, m_robot.m_robotTransport)
                ),
                new AutoAim(m_robot.m_robotDrive, m_robot.m_robotSpinner, m_robot.m_vision).getCommand(),
                new TimedTurret(m_robot.m_robotTurret, 1.5, 1750)));
    }
}
