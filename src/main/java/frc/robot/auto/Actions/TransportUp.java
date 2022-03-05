// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.Actions;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Transporter;

public class TransportUp extends CommandBase {
  Transporter m_transporter;
  Intake m_intake;
  Timer timer = new Timer();
  private double s;
  private double t;

  public TransportUp(Intake intake, Transporter robotTransporter) {
    m_transporter = robotTransporter;
    m_intake = intake;
    s = 0.25;
    t = 1.5;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_transporter);
    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      timer.reset();
      timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_transporter.transportRun(s);
    m_intake.intakeRun();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_transporter.transportStop();
    m_intake.intakeStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (timer.get() > t) {
        m_transporter.transportRun(0);
        m_intake.intakeStop();
        return true;
    }
    return false;
  }

  public void interrupted() {
    m_transporter.transportRun(0);
    m_intake.intakeStop();
  }
}