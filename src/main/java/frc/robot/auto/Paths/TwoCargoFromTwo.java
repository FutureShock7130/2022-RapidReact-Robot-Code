package frc.robot.auto.Paths;

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
import frc.robot.auto.Actions.TransportBoost;
import frc.robot.auto.Actions.TransportUp;
import frc.robot.commands.Drive.AbsoluteAim;
import frc.robot.commands.Intake.IntakeCmd;
import frc.robot.commands.Intake.TimedIntake;
import frc.robot.commands.Transporter.TimedTransport;
import frc.robot.commands.Turret.TimedTurret;

public class TwoCargoFromTwo {
    TrajectoryGenerator trajectoryGenerator;
    RobotContainer m_robot;

    public TwoCargoFromTwo(RobotContainer robot) {
        m_robot = robot;
        trajectoryGenerator = new TrajectoryGenerator(m_robot.m_robotDrive);
    }

    public SequentialCommandGroup getCommand() {
        return new SequentialCommandGroup(
                new AutoAim(m_robot.m_robotDrive, m_robot.m_robotSpinner, m_robot.m_vision).getCommand(),
                new ParallelCommandGroup(
                        new TimedTurret(m_robot.m_robotTurret, 3.0, 1800),
                        new SequentialCommandGroup(
                                new WaitCommand(1.0),
                                new TransportUp(m_robot.m_robotIntake, m_robot.m_robotTransport)
                        )
                ),
                new AbsoluteAim(m_robot.m_robotDrive, true, 0.9396, -0.342),
                new ParallelRaceGroup(
                        trajectoryGenerator.generateTranslationalPrimary(
                                "(2) 2nd Cargo Straight", 
                                new PIDController(1.5, 0.0003, 0),
                                new PIDController(1.5, 0.0003, 0)
                        ),
                        new IntakeCmd(m_robot.m_robotIntake)
                ),
                new AbsoluteAim(m_robot.m_robotDrive, true, 0.8660, 0.5),
                new ParallelCommandGroup(
                        trajectoryGenerator.generateTranslationalPrimary(
                                "(2) 3rd Cargo", 
                                new PIDController(1.5, 0.0003, 0),
                                new PIDController(1.5, 0.0003, 0)
                        ),
                        new TimedIntake(3.0, m_robot.m_robotIntake)
                ),
                new AbsoluteAim(m_robot.m_robotDrive, true, 0, -1),
                trajectoryGenerator.generate(
                        "(2) 3rd Cargo to Shooting Position", 
                        new PIDController(1.5, 0.0003, 0),
                        new PIDController(1.5, 0.0003, 0),
                        new ProfiledPIDController(
                                0.5, 0, 0, 
                                new TrapezoidProfile.Constraints(3.5, 2.5))
                ),
                new AbsoluteAim(m_robot.m_robotDrive, true, 0.707, 0.707),
                new AutoAim(m_robot.m_robotDrive, m_robot.m_robotSpinner, m_robot.m_vision).getCommand(),
                new ParallelCommandGroup(
                        new TimedTurret(m_robot.m_robotTurret, 3.0, 1800),
                        new SequentialCommandGroup(
                                new TransportUp(m_robot.m_robotIntake, m_robot.m_robotTransport),
                                new WaitCommand(1.0),
                                new TransportUp(m_robot.m_robotIntake, m_robot.m_robotTransport)
                        )
                )            
        );   
    }
}
