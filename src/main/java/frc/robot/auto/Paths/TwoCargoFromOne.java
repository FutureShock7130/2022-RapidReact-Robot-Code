package frc.robot.auto.Paths;

import javax.xml.namespace.QName;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.Intake.TimedIntake;
import frc.robot.commands.Transporter.TimedTransport;
import frc.robot.commands.Turret.TimedTurret;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Transporter;
import frc.robot.subsystems.Turret;

public class TwoCargoFromOne {
    Intake m_robotIntake;
    TrajectoryGenerator trajectoryGenerator;
    Transporter m_robotTransport;
    Drive m_robotDrive;
    Turret m_robotTurret;

    public TwoCargoFromOne(RobotContainer robot, Transporter m_robotTransport, Intake m_robotIntake, Drive m_robotDrive,
            Turret m_robotTurret) {
        this.m_robotTransport = m_robotTransport;
        this.m_robotTransport = m_robotTransport;
        this.m_robotDrive = m_robotDrive;
        this.m_robotIntake = m_robotIntake;
        this.m_robotTurret = m_robotTurret;
        trajectoryGenerator = new TrajectoryGenerator(this.m_robotDrive);
    }

    public SequentialCommandGroup getCommand() {
        return new SequentialCommandGroup(

                new ParallelCommandGroup(
                        trajectoryGenerator.generateTranslationalPrimary("1 to 2 from 1",
                                new PIDController(0.03, 0.003, 0.003), new PIDController(0.03, 0.003, 0.003)),
                        new TimedIntake(2.0, this.m_robotIntake)),

                new ParallelCommandGroup(
                        trajectoryGenerator.generateTranslationalPrimary("2 to 2s from 1",
                                new PIDController(0.03, 0.003, 0.003), new PIDController(0.03, 0.003, 0.003)),
                        new SequentialCommandGroup(
                                new TimedIntake(0.5, this.m_robotIntake),
                                new WaitCommand(0.5),
                                new TimedTransport(1, this.m_robotTransport)),
                        new TimedTurret(this.m_robotTurret, 1, 1750)));
    }
}
