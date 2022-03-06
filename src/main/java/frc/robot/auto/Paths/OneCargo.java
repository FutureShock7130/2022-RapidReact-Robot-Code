package frc.robot.auto.Paths;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.MecanumControllerCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.auto.Actions.AutoAim;
import frc.robot.auto.Actions.TransportUp;
import frc.robot.commands.Transporter.TimedTransport;
import frc.robot.commands.Turret.TimedTurret;
import frc.robot.commands.Turret.TurretShoot;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class OneCargo {

    private MecanumControllerCommand route;
    private PIDController xController = DriveConstants.idealXController;
    private PIDController yController = DriveConstants.idealYController;
    private ProfiledPIDController thetaController = DriveConstants.idealThetaController;

    RobotContainer m_robot;

    private AutoAim AutoAim;
    public OneCargo(RobotContainer robot) {
        m_robot = robot;

        AutoAim = new AutoAim(m_robot.m_robotDrive, m_robot.m_robotSpinner, m_robot.m_vision);
    }

    public SequentialCommandGroup getCommand() {
        return new SequentialCommandGroup(
                AutoAim.getCommand().withTimeout(7.0),
                new ParallelDeadlineGroup(
                        new SequentialCommandGroup(
                            new WaitCommand(0.5), 
                            new TransportUp(m_robot.m_robotIntake, m_robot.m_robotTransport).withTimeout(1.5)
                        ),
                        new TurretShoot(m_robot.m_robotTurret, 1550).withTimeout(4.0)
                    ),
                new RunCommand(
                    () -> {
                    m_robot.m_robotDrive.drive(-0.3, -0.3, 0, false);
                    }, m_robot.m_robotDrive).withTimeout(2)
        );
    }       

}
