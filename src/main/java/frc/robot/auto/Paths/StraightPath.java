package frc.robot.auto.Paths;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.auto.Actions.AutoAim;
import frc.robot.auto.Actions.TransportUp;
import frc.robot.commands.Drive.AbsoluteAim;
import frc.robot.commands.Intake.IntakeCmd;
import frc.robot.commands.Intake.TimedIntake;
import frc.robot.commands.Transporter.TimedTransport;
import frc.robot.commands.Transporter.TransportCmd;
import frc.robot.commands.Turret.TimedTurret;
import frc.robot.commands.Turret.TurretShoot;

public class StraightPath {
    TrajectoryGenerator trajectoryGenerator;
    RobotContainer m_robot;
    String initialPathName;

    public StraightPath(RobotContainer robot) {
        m_robot = robot;
        trajectoryGenerator = new TrajectoryGenerator(m_robot.m_robotDrive);
        initialPathName = "(1) 2nd Cargo";
    }

    public SequentialCommandGroup getCommand() {
        return new SequentialCommandGroup(
            trajectoryGenerator.generateTranslationalPrimary(
                "Straight Test Path", 
                new PIDController(1.3, 0.0003, 0),
                new PIDController(1.4, 0.0003, 0)
            )
        );
    }
}
