package frc.robot.auto.Paths;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.MecanumControllerCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.auto.Actions.AutoAim;
import frc.robot.commands.Transporter.TimedTransport;
import frc.robot.commands.Turret.TimedTurret;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class OneCargo {

    private MecanumControllerCommand route;
    private PIDController xController = DriveConstants.idealXController;
    private PIDController yController = DriveConstants.idealYController;
    private ProfiledPIDController thetaController = DriveConstants.idealThetaController;

    RobotContainer m_robot;

    private AutoAim AutoAim;
    private TrajectoryGenerator trajectoryGenerator;

    public OneCargo(RobotContainer robot) {
        m_robot = robot;
        trajectoryGenerator = new TrajectoryGenerator(m_robot.m_robotDrive);

        AutoAim = new AutoAim(m_robot.m_robotDrive, m_robot.m_robotSpinner, m_robot.m_vision);
        //route = trajectoryGenerator.generate("", xController, yController, thetaController);
    }

    public SequentialCommandGroup getCommand() {
        return new SequentialCommandGroup(
                AutoAim.getCommand(),
                new ParallelDeadlineGroup(
                        new SequentialCommandGroup(new WaitCommand(0.5), new TimedTransport(1, m_robot.m_robotTransport)),
                        new TimedTurret(m_robot.m_robotTurret, 1, 1750)));
    }

}
