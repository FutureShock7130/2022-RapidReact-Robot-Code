package frc.robot.auto.Actions.Plan_B_test;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Spinner;
import frc.robot.subsystems.Turret;
import frc.robot.vision.Limelight;

public class Path_B1 extends CommandBase {

  Drive drive;
  Intake intake;
  Spinner spinner;
  Limelight limelight;
  Turret turret;

  private double currentPositionMeter;
  private double targetPositionMeter;
  private double errorPostionMeter;
  private double currentAngleDegrees;
  private double targetAngleDegrees;
  private double errorAngleDegrees;
  private double initTime;

  public Path_B1(Drive m_drive, Intake m_robotIntake, Spinner m_robotSpinner, Limelight m_robotLimelight,
      Turret m_robotTurret) {
    drive = m_drive;
    intake = m_robotIntake;
    spinner = m_robotSpinner;
    limelight = m_robotLimelight;
    turret = m_robotTurret;

    addRequirements(drive);
    addRequirements(intake);
    addRequirements(limelight);
    addRequirements(spinner);
    addRequirements(turret);
  }

  @Override
  public void initialize() {
    drive.resetEncoders();
    drive.setGyroZeroYaw();
  }

  @Override
  public void execute() {

    // going backward 1.1 m
    targetPositionMeter = 1.1;
    while (errorPostionMeter <= 0.05) {
      drive.drive(errorPostionMeter * 0.2, 0, 0, false);
      currentPositionMeter = drive.getAverageWheelPosition();
      errorPostionMeter = targetPositionMeter - currentPositionMeter;
    }
    drive.resetEncoders();
    drive.setGyroZeroYaw();

    // left turn 56 deg
    currentAngleDegrees = 0;
    targetAngleDegrees = 56;
    while (errorAngleDegrees <= 5) {
      drive.drive(0, 0, -errorAngleDegrees * 0.2, false);
      currentAngleDegrees = drive.getHeading();
      errorAngleDegrees = targetAngleDegrees - currentAngleDegrees;
    }
    drive.resetEncoders();
    drive.setGyroZeroYaw();

    // going forward 0.6 m and intake
    targetPositionMeter = 0.6;
    while (errorPostionMeter <= 0.05) {
      drive.drive(-errorPostionMeter * 0.2, 0, 0, false);
      currentPositionMeter = drive.getAverageWheelPosition();
      errorPostionMeter = targetPositionMeter - currentPositionMeter;
    }
    drive.resetEncoders();
    drive.setGyroZeroYaw();

    initTime = Timer.getFPGATimestamp();
    while (initTime >= Timer.getFPGATimestamp() - 2) {
      intake.intakeRun();
    }

    // spinner aim limelight and shoot 2 balls
    initTime = Timer.getFPGATimestamp();
    while (initTime >= Timer.getFPGATimestamp() - 1) { // time adjust
      spinner.spinnerRun(0.2); // velocity still need to adjust
    }

    initTime = Timer.getFPGATimestamp();
    while (initTime >= Timer.getFPGATimestamp() - 2.5) { // time adjust
      turret.flywheelsRun(0.4); // veclocity still need to adjust
    }

    // going forward 2.62 m and intake
    targetPositionMeter = 2.62;
    while (errorPostionMeter <= 0.05) {
      drive.drive(-errorPostionMeter * 0.2, 0, 0, false);
      currentPositionMeter = drive.getAverageWheelPosition();
      errorPostionMeter = targetPositionMeter - currentPositionMeter;
    }
    drive.resetEncoders();
    drive.setGyroZeroYaw();

    // right turn 90 deg
    currentAngleDegrees = 0;
    targetAngleDegrees = 90;
    while (errorAngleDegrees <= 5) {
      drive.drive(0, 0, errorAngleDegrees * 0.2, false);
      currentAngleDegrees = drive.getHeading();
      errorAngleDegrees = targetAngleDegrees - currentAngleDegrees;
    }
    drive.resetEncoders();
    drive.setGyroZeroYaw();

    // spinner aim limelight and shoot 1 balls

    // turn left 133 deg
    currentAngleDegrees = 0;
    targetAngleDegrees = 133;
    while (errorAngleDegrees <= 5) {
      drive.drive(0, 0, -errorAngleDegrees * 0.2, false);
      currentAngleDegrees = drive.getHeading();
      errorAngleDegrees = targetAngleDegrees - currentAngleDegrees;
    }
    drive.resetEncoders();
    drive.setGyroZeroYaw();
  }
}