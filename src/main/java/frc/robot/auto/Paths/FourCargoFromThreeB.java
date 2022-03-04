package frc.robot.auto.Paths;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.MecanumControllerCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.auto.Actions.AutoAim;
import frc.robot.commands.Intake.TimedIntake;
import frc.robot.commands.Transporter.TimedTransport;
import frc.robot.commands.Turret.TimedTurret;

public class FourCargoFromThreeB {

    private MecanumControllerCommand route;

    RobotContainer m_robot;

    private PIDController xController = DriveConstants.idealXController;
    private PIDController yController = DriveConstants.idealYController;
    private ProfiledPIDController thetaController = DriveConstants.idealThetaController;

    private TrajectoryGenerator trajectoryGenerator;

    public FourCargoFromThreeB(RobotContainer robot) {
        m_robot = robot;
        trajectoryGenerator = new TrajectoryGenerator(m_robot.m_robotDrive);
        route = trajectoryGenerator.generate("2S to 4S from 3", xController, yController, thetaController);
    }

    public SequentialCommandGroup getCommand() {
        SequentialCommandGroup lastCommand = new TwoCargoFromThree(m_robot).getCommand();
        return new SequentialCommandGroup(
                lastCommand,
                new ParallelDeadlineGroup(route,
                        new SequentialCommandGroup(new WaitCommand(1), new TimedIntake(3, m_robot.m_robotIntake))),

                new ParallelCommandGroup(
                        new TimedTurret(m_robot.m_robotTurret, 1.5, 1750),
                        new AutoAim(m_robot.m_robotDrive, m_robot.m_robotSpinner, m_robot.m_vision).getCommand(),
                        new SequentialCommandGroup(new WaitCommand(0.5),
                                new ParallelCommandGroup(new TimedIntake(1, m_robot.m_robotIntake),
                                        new TimedTransport(1, m_robot.m_robotTransport)))));
    }
}
