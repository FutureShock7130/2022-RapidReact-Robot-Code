// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;

public class TimedIntake extends CommandBase {
  Intake intake;
  Timer timer = new Timer();
  private double t;

  public TimedIntake(double time, Intake m_robotIntake) {
    intake = m_robotIntake;
    t = time;

    addRequirements(intake);
    
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
    intake.intakeRun();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.intakeStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (timer.get() > t){
      intake.intakeStop();
      return true;
    }
    return false;

  }
}