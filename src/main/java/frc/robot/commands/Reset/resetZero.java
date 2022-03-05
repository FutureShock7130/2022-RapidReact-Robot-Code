
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Reset;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Spinner;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Transporter;
import frc.robot.subsystems.Turret;
import frc.robot.vision.Limelight;

public class ResetZero extends CommandBase {
  Drive drive;
  Limelight limelight;
  Intake intake;
  Transporter transporter;
  Spinner spinner;
  Turret turret;
  Superstructure superstructure;

  /** Creates a new resetZero. */
  public ResetZero(RobotContainer robot) {

    drive = robot.m_robotDrive;
    limelight = robot.m_vision;
    intake = robot.m_robotIntake;
    transporter = robot.m_robotTransport;
    spinner = robot.m_robotSpinner;
    turret = robot.m_robotTurret;
    superstructure = robot.m_SuperStructure;

    
    // Use addRequirements() here to declare subsystem dependencies.
     addRequirements(drive);
     addRequirements(limelight);
     addRequirements(intake);
     addRequirements(transporter);
     addRequirements(spinner);
     addRequirements(turret);
     addRequirements(superstructure);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.resetEncoders();
    drive.setGyroZeroYaw();
    intake.intakeStop();
    spinner.spinnerStop();
    superstructure.liftHangerStop();
    superstructure.liftSwingStop();
    transporter.transportStop();
    turret.flywheelsStop();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
