package frc.robot.auto.Paths;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.auto.Actions.AutoAim;
import frc.robot.auto.Actions.TransportUp;
import frc.robot.commands.Turret.TurretShoot;

public class TurretTest {
    TrajectoryGenerator trajectoryGenerator;
    RobotContainer m_robot;
    String initialPathName;

    public TurretTest(RobotContainer robot) {
        m_robot = robot;
        trajectoryGenerator = new TrajectoryGenerator(m_robot.m_robotDrive);
        initialPathName = "(1) 2nd Cargo";
    }

    public SequentialCommandGroup getCommand() {
        return new SequentialCommandGroup(
                new AutoAim(m_robot.m_robotDrive, m_robot.m_robotSpinner, m_robot.m_vision).getCommand().withTimeout(4.0),
                new ParallelCommandGroup(
                        new TurretShoot(m_robot.m_robotTurret, 1500).withTimeout(4.0),
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new TransportUp(m_robot.m_robotIntake, m_robot.m_robotTransport),
                                        new WaitCommand(0.5),
                                        new TransportUp(m_robot.m_robotIntake, m_robot.m_robotTransport),
                                        new WaitCommand(0.5),
                                        new TransportUp(m_robot.m_robotIntake, m_robot.m_robotTransport)
                                )
                        )
                ).withTimeout(4.0)
        );         
    }
}
