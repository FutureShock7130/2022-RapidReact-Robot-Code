// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Transporter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.TransporterConstants;
import frc.robot.subsystems.Transporter;

public class TimedTransport extends CommandBase {
  Transporter transporter;
  Timer timer = new Timer();
  private double voltage;

  /** Creates a new intakeCmd. */
  public TimedTransport(double speed, Transporter m_robotTransporter) {
    transporter = m_robotTransporter;
    voltage = speed;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(transporter);
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
    transporter.transportRun(voltage);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    transporter.transportRun(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (timer.get() > TransporterConstants.idealTransportDt) {
        transporter.transportRun(0);
        return true;
    }
    return false;
  }

  public void interrupted() {
    transporter.transportRun(0);
  }
}