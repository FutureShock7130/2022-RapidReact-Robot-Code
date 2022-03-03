package frc.robot.auto.Paths;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.command.TimedCommand;
import edu.wpi.first.wpilibj2.command.MecanumControllerCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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

public class ThreeCargoFromThree {

    private MecanumControllerCommand route1;
    private MecanumControllerCommand route2;
    private MecanumControllerCommand route3;

    private Transporter m_robotTransport;
    private Turret m_robotTurret;
    private Drive m_robotDrive;
    private Limelight m_vision;
    private Spinner m_robotSpinner;
    private Intake m_robotIntake;

    private PIDController xController = DriveConstants.idealXController;
    private PIDController yController = DriveConstants.idealYController;
    private ProfiledPIDController thetaController = DriveConstants.idealThetaController;

    private TrajectoryGenerator trajectoryGenerator;

    public ThreeCargoFromThree(Drive m_robotDrive, Intake m_robotIntake, Spinner m_robotSpinner,
            Transporter m_robotTransport) {
        this.m_robotDrive = m_robotDrive;
        this.m_robotIntake = m_robotIntake;
        this.m_robotSpinner = m_robotSpinner;
        this.m_robotTransport = m_robotTransport;
        trajectoryGenerator = new TrajectoryGenerator(this.m_robotDrive);
        route1 = trajectoryGenerator.generate("2 to 3 from 3 copy", xController, yController, thetaController);
        route2 = trajectoryGenerator.generate("4 to 5 from 3 copy", xController, yController, thetaController);
        route3 = trajectoryGenerator.generate("5 to 6 from 3 copy", xController, yController, thetaController);
    }

    public SequentialCommandGroup getCommand() {
        return new SequentialCommandGroup(
                new ParallelDeadlineGroup(route1,
                        new SequentialCommandGroup(new WaitCommand(1), new TimedIntake(3, m_robotIntake))),
                new ParallelCommandGroup(
                        new TimedTurret(m_robotTurret, 1.5, 2050),
                        new AutoAim(m_robotDrive, m_robotSpinner, m_vision).getCommand(),
                        new SequentialCommandGroup(new WaitCommand(0.5),
                                new ParallelCommandGroup(new TimedIntake(1, m_robotIntake),
                                        new TimedTransport(1, m_robotTransport)))),
                new ParallelDeadlineGroup(route2,
                        new SequentialCommandGroup(new WaitCommand(1.5), new TimedIntake(3, m_robotIntake))),
                route3,
                new ParallelCommandGroup(
                        new TimedTurret(m_robotTurret, 1.5, 2050),
                        new AutoAim(m_robotDrive, m_robotSpinner, m_vision).getCommand(),
                        new SequentialCommandGroup(new WaitCommand(0.5),
                                new ParallelCommandGroup(new TimedIntake(1, m_robotIntake),
                                        new TimedTransport(1, m_robotTransport)))));
    }
}
