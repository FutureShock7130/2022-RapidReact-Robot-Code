package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {

  private final WPI_TalonSRX intaker = new WPI_TalonSRX(IntakeConstants.kintakeID);;

  /** Creates a new IntakeSubsystem. */
  public Intake() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void intakeRun(){
    intaker.set(IntakeConstants.intakeSpeed);
  }

  public void intakeStop(){
    intaker.set(0);
  }

  
}
