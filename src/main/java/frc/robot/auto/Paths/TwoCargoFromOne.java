package frc.robot.auto.Paths;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.auto.AutoModes;
import frc.robot.auto.Actions.AutoAim;
import frc.robot.commands.Intake.TimedIntake;
import frc.robot.commands.Turret.TimedTurret;

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
                trajectoryGenerator.generateTranslationalPrimary("1 to 2 from 1", new PIDController(0.03, 0.003, 0.003), new PIDController(0.03, 0.003, 0.003)),
                new TimedIntake(2.0, m_robot.m_robotIntake)
            ),
            trajectoryGenerator.generate("2 to 2S from 1",
                new PIDController(3, 0, 0.03),
                new PIDController(3, 0, 0.03),
                new ProfiledPIDController(1.4, 0.0004, 0.014,
                    new TrapezoidProfile.Constraints(Math.PI, Math.PI / 2)
                )
            ),
            new AutoAim(m_robot.m_robotDrive, m_robot.m_robotSpinner, m_robot.m_vision).getCommand(),
            new TimedTurret(m_robot.m_robotTurret, 1.5, 1800)
        );
    }
}
