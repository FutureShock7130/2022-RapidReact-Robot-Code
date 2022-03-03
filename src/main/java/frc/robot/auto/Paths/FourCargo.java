package frc.robot.auto.Paths;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.auto.Actions.AutoAim;
import frc.robot.commands.Intake.IntakeCmd;
import frc.robot.commands.Transporter.TimedTransport;
import frc.robot.commands.Turret.TimedTurret;

public class FourCargo {

    private RobotContainer m_robot;

    public FourCargo(RobotContainer m_robot) {
        this.m_robot = m_robot;
    }

    public SequentialCommandGroup getCommand(){
        return new SequentialCommandGroup(
            new ParallelDeadlineGroup(
                new TrajectoryGenerator(m_robot.m_robotDrive).generate("2S to 4 from 1 B", DriveConstants.idealXController, DriveConstants.idealYController, DriveConstants.idealThetaController),
                new IntakeCmd(m_robot.m_robotIntake)
            ),
            new AutoAim(m_robot.m_robotDrive, m_robot.m_robotSpinner, m_robot.m_vision).getCommand(),
            new ParallelCommandGroup(new TimedTransport(1, m_robot.m_robotTransport), new TimedTurret(m_robot.m_robotTurret, 1, 1750))
        );
    }

}
