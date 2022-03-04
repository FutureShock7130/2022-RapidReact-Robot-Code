// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Transporter;

public class TransportToShoot extends CommandBase {

  Intake intake;
  Transporter transporter;

  Timer timer = new Timer();
  
  /** Creates a new intakeCmd. */
  public TransportToShoot(Intake m_robotIntake, Transporter m_transporter) {
    intake = m_robotIntake;
    transporter = m_transporter;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
    addRequirements(transporter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.downRun();
    transporter.transportRun(0.6);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.intakeStop();
    transporter.transportStop();
    System.out.println("IntakeCmd ended");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (timer.get() > 0.5) {
        intake.downStop();
        transporter.transportStop();
        return true;
    }
    return false;
  }

  public void interrupted(){
    intake.intakeStop();
    transporter.transportStop();
  }
}