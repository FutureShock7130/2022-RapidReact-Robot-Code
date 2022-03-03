package frc.robot.auto.Paths;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
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

public class ThreeCargoFromOne {

    private MecanumControllerCommand route1;
    private MecanumControllerCommand route2;
    
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
    
    public ThreeCargoFromOne(Drive m_robotDrive, Intake m_robotIntake, Spinner m_robotSpinner, Transporter m_robotTransport){
        this.m_robotDrive = m_robotDrive;
        this.m_robotIntake = m_robotIntake;
        this.m_robotSpinner = m_robotSpinner;
        this.m_robotTransport = m_robotTransport;
        trajectoryGenerator = new TrajectoryGenerator(this.m_robotDrive);
        route1 = trajectoryGenerator.generate("2S to 3 from 1", xController, yController, thetaController);
        route2 = trajectoryGenerator.generate("3 to 3S from 1 A", xController, yController, thetaController);
    }

    public SequentialCommandGroup getCommand(){
        return new SequentialCommandGroup(
            new ParallelDeadlineGroup(route1, new SequentialCommandGroup(new WaitCommand(1), new TimedIntake(1, m_robotIntake))),
            route2,
            new AutoAim(this.m_robotDrive, this.m_robotSpinner, this.m_vision).getCommand(),
            new ParallelCommandGroup(new TimedTransport(1, this.m_robotTransport), new TimedTurret(this.m_robotTurret, 1, 1750))
        );
    }

}
