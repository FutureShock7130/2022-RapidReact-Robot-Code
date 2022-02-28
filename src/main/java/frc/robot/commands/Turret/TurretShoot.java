// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Turret;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

public class TurretShoot extends CommandBase {

  private static final double kP = 0.0005;
  private static final double kI = 0.0002;
  // a D Controller is not needed for the basic flywheel control because we only
  // need to rev the spin speed up instead of it being
  // reving up and down gradually.
  private static final double kD = 0;
  private static final double timeDiff = 0.02;

  private double target = 1500;
  private double integralSum;
  private double derivative;
  private double error;
  private double lastError;
  private double output;

  Turret turret;

  /** Creates a new TurretShoot. */
  public TurretShoot(Turret m_robotTurret) {
    turret = m_robotTurret;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    error = target - turret.getFlyWheelsVelocity();

    if (Math.abs(integralSum) < 100) {
      integralSum += error;
    }

    derivative = (error - lastError) / timeDiff;
    output = kP * error + kI * integralSum + kD * derivative;

    // The Conditional Loop below tries to implement PID with Bang-Bang control,
    // this ensures that when large errors occur, the
    // Flywheel should rev up quickly
    if (error > 500) {
      turret.flywheelsRun(1.0);
    } else if (error < -500) {
      turret.flywheelsRun(-1.0);
    } else {
      turret.flywheelsRun(output);
    }

    lastError = error;

    SmartDashboard.putNumber("velocity", turret.getFlyWheelsVelocity());
    SmartDashboard.putNumber("Turret Output", output);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.flywheelsStop();
  }

  public void interrupted() {
    turret.flywheelsRun(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
