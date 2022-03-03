package frc.robot.auto.Paths;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.command.TimedCommand;
import edu.wpi.first.wpilibj2.command.MecanumControllerCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.auto.Actions.AutoAim;
import frc.robot.commands.Intake.TimedIntake;
import frc.robot.commands.Transporter.TimedTransport;
import frc.robot.commands.Turret.TimedTurret;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Spinner;
import frc.robot.subsystems.Transporter;
import frc.robot.subsystems.Turret;
import frc.robot.vision.Limelight;

public class ThreeCargoFromTwo {

    private MecanumControllerCommand route1;
    private MecanumControllerCommand route2;
    private MecanumControllerCommand route3;
    private MecanumControllerCommand route4;

    RobotContainer m_robot;

    private PIDController xController = DriveConstants.idealXController;
    private PIDController yController = DriveConstants.idealYController;
    private ProfiledPIDController thetaController = DriveConstants.idealThetaController;

    private TrajectoryGenerator trajectoryGenerator;

    public ThreeCargoFromTwo(RobotContainer robot) {
        m_robot = robot;
        trajectoryGenerator = new TrajectoryGenerator(m_robot.m_robotDrive);
        route1 = trajectoryGenerator.generate("1 to 2 from 2", xController, yController, thetaController);
        route2 = trajectoryGenerator.generate("2 to 2s from 2", xController, yController, thetaController);
        route3 = trajectoryGenerator.generate("2s to 3 from 2", xController, yController, thetaController);
        route4 = trajectoryGenerator.generate("3 to 3s from 2 (new)", xController, yController, thetaController);
    }

    public SequentialCommandGroup getCommand() {
        return new SequentialCommandGroup(
                new ParallelDeadlineGroup(route1,
                        new SequentialCommandGroup(new WaitCommand(1), new TimedIntake(1, m_robot.m_robotIntake))),
                route2,
                new ParallelCommandGroup(
                        new TimedTurret(m_robot.m_robotTurret, 1.5, 1750),
                        new AutoAim(m_robot.m_robotDrive, m_robot.m_robotSpinner, m_robot.m_vision).getCommand(),
                        new SequentialCommandGroup(new WaitCommand(0.5),
                                new ParallelCommandGroup(new TimedIntake(1, m_robot.m_robotIntake),
                                        new TimedTransport(1, m_robot.m_robotTransport)))),
                new ParallelDeadlineGroup(route3,
                        new SequentialCommandGroup(new WaitCommand(1), new TimedIntake(2.5, m_robot.m_robotIntake))),
                route4,
                new ParallelCommandGroup(
                        new TimedTurret(m_robot.m_robotTurret, 1.5, 1750),
                        new AutoAim(m_robot.m_robotDrive, m_robot.m_robotSpinner, m_robot.m_vision).getCommand(),
                        new SequentialCommandGroup(new WaitCommand(0.5),
                                new ParallelCommandGroup(new TimedIntake(1, m_robot.m_robotIntake),
                                        new TimedTransport(1, m_robot.m_robotTransport)))));
    }
}
