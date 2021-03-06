// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Transporter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.TransporterConstants;
import frc.robot.subsystems.Transporter;
   
public class TransportEject extends CommandBase {
  Transporter transporter;

  /** Creates a new intakeCmd. */
  public TransportEject(Transporter m_robotTransporter) {
    transporter = m_robotTransporter;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(transporter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    transporter.transportEject(TransporterConstants.transportSpeed);
  } 

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
