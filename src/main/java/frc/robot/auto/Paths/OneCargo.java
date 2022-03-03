package frc.robot.auto.Paths;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.MecanumControllerCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveConstants;
import frc.robot.auto.Actions.AutoAim;
import frc.robot.commands.Transporter.TimedTransport;
import frc.robot.commands.Turret.TimedTurret;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Spinner;
import frc.robot.subsystems.Transporter;
import frc.robot.subsystems.Turret;
import frc.robot.vision.Limelight;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class OneCargo {

    private Transporter m_robotTransport;
    private Turret m_robotTurret;
    private Drive m_robotDrive;
    private Limelight m_vision;
    private Spinner m_robotSpinner;

    private MecanumControllerCommand route;
    private PIDController xController = DriveConstants.idealXController;
    private PIDController yController = DriveConstants.idealYController;
    private ProfiledPIDController thetaController = DriveConstants.idealThetaController;

    private AutoAim AutoAim;
    private TrajectoryGenerator trajectoryGenerator;

    public OneCargo(Intake m_robotIntake, Transporter m_robotTransport, Turret m_robotTurret, Drive m_robotDrive,
            Limelight m_vision, Spinner m_robotSpinner) {
        this.m_robotTurret = m_robotTurret;
        this.m_robotTransport = m_robotTransport;
        this.m_robotDrive = m_robotDrive;
        this.m_vision = m_vision;
        this.m_robotSpinner = m_robotSpinner;
        this.trajectoryGenerator = new TrajectoryGenerator(this.m_robotDrive);

        AutoAim = new AutoAim(this.m_robotDrive, this.m_robotSpinner, this.m_vision);
        route = trajectoryGenerator.generate("One", this.xController, this.yController, this.thetaController);
    }

    public SequentialCommandGroup getCommand() {
        return new SequentialCommandGroup(
                route,
                AutoAim.getCommand(),
                new ParallelDeadlineGroup(
                        new SequentialCommandGroup(new WaitCommand(0.5), new TimedTransport(1, m_robotTransport)),
                        new TimedTurret(m_robotTurret, 1, 1750)));
    }

}
