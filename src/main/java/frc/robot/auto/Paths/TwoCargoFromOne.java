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
import frc.robot.auto.Actions.TransportUp;
import frc.robot.commands.Drive.AbsoluteAim;
import frc.robot.commands.Intake.IntakeCmd;
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
                new AbsoluteAim(m_robot.m_robotDrive, true, 0, -1),
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
