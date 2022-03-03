package frc.robot.auto.Paths;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.Intake.TimedIntake;

public class TwoCargoFromOne {
    RobotContainer m_robot;
    TrajectoryGenerator trajectoryGenerator;

    public TwoCargoFromOne(RobotContainer robot) {
        m_robot = robot;
        trajectoryGenerator = new TrajectoryGenerator(m_robot.m_robotDrive);
    }

    public SequentialCommandGroup getCommand() {
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        trajectoryGenerator.generateTranslationalPrimary("1 to 2 from 1",
                                new PIDController(0.03, 0.003, 0.003), new PIDController(0.03, 0.003, 0.003)),
                        new TimedIntake(2.0, m_robot.m_robotIntake)));
    }

}
