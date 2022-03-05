// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Turret;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.Turret;

public class TurretShoot extends CommandBase {
  private static final double kP = 0.0013;
  private static final double kI = 0.00002; //0.00002

  // a D Controller is not needed for the basic flywheel control because we only
  // need to rev the spin speed up instead of it being reving up and dow
  // gradually.

  private static final double kD = 0.00015;
  private static final double timeDiff = 0.02;

  private double target;
  private double integralSum;

  private double derivative;
  private double error;
  private double lastError;
  private double output;

  Turret turret;

  private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(TurretConstants.kV, TurretConstants.kA);

  /** Creates a new TurretShoot. */
  public TurretShoot(Turret m_robotTurret, double targetSet) {
    target = targetSet;
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
    double kF = feedforward.calculate(target);

    error = target - turret.getFlyWheelsVelocity();

    if (Math.abs(integralSum) < 40000) {
      integralSum += error;
    }

    derivative = (error - lastError) / timeDiff;
    output = kP * error + kI * integralSum + kD * derivative;
    SmartDashboard.putNumber("error", error);

    // The Conditional Loop below tries to implement PID with Bang-Bang control,
    // this ensures that when large errors occur, the
    // Flywheel should rev up quickly

    if (error > 500) {
      turret.flywheelsRun(1.0);
    } else { 
      turret.flywheelsRun(output);
    }

    lastError = error;

    SmartDashboard.putNumber("Flywheel Velocity", turret.getFlyWheelsVelocity());
    SmartDashboard.putNumber("Flywheel Voltage Output", output);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.flywheelsRun(0);
  }

  public void interrupted() {
    turret.flywheelsRun(0);
  }

  // Returns true when the command should end.
  // For this command, this is perpetual.
  @Override
  public boolean isFinished() {
    return false;
  }
}
