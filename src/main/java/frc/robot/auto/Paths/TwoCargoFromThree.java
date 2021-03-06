package frc.robot.auto.Paths;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.MecanumControllerCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.auto.Actions.AutoAim;
import frc.robot.auto.Actions.TransportUp;
import frc.robot.commands.Drive.AbsoluteAim;
import frc.robot.commands.Intake.IntakeCmd;
import frc.robot.commands.Intake.TimedIntake;
import frc.robot.commands.Transporter.TimedTransport;
import frc.robot.commands.Turret.TimedTurret;
import frc.robot.commands.Turret.TurretShoot;
public class TwoCargoFromThree {

    private MecanumControllerCommand route;

    RobotContainer m_robot;

    private PIDController xController = DriveConstants.idealXController;
    private PIDController yController = DriveConstants.idealYController;
    private ProfiledPIDController thetaController = DriveConstants.idealThetaController;

    private TrajectoryGenerator trajectoryGenerator;

    public TwoCargoFromThree(RobotContainer robot) {
        m_robot = robot;
        trajectoryGenerator = new TrajectoryGenerator(m_robot.m_robotDrive);
    }

    public SequentialCommandGroup getCommand() {
        return new SequentialCommandGroup(
                new ParallelRaceGroup(
                        trajectoryGenerator.generateTranslationalPrimary(
                                "Back Pickup", 
                                new PIDController(1.3, 0.0003, 0),
                                new PIDController(1.3, 0.0003, 0)
                        ),
                        new IntakeCmd(m_robot.m_robotIntake)
                ),
                new AbsoluteAim(m_robot.m_robotDrive, true, -180),
                trajectoryGenerator.generateTranslationalPrimary(
                                "Back Pickup", 
                                new PIDController(1.3, 0.0003, 0),
                                new PIDController(1.3, 0.0003, 0)
                ),
                new AutoAim(m_robot.m_robotDrive, m_robot.m_robotSpinner, m_robot.m_vision).getCommand().withTimeout(3.0),
                new ParallelCommandGroup(
                        new TurretShoot(m_robot.m_robotTurret, 1550).withTimeout(3.0),
                        new SequentialCommandGroup(
                                new WaitCommand(0.5),
                                new TransportUp(m_robot.m_robotIntake, m_robot.m_robotTransport),
                                new WaitCommand(0.5),
                                new TransportUp(m_robot.m_robotIntake, m_robot.m_robotTransport)
                        )
                )            
        );   
    }
}
