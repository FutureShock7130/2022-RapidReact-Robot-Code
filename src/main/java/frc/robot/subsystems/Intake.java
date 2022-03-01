package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
//import frc.robot.Constants.TransporterConstants;
import frc.robot.Constants.TransporterConstants;

public class Intake extends SubsystemBase {

  private final WPI_TalonSRX intaker = new WPI_TalonSRX(IntakeConstants.kIntakeID);;
  private final WPI_TalonSRX downTransporter = new WPI_TalonSRX(TransporterConstants.kDownTransporterID);

  /** Creates a new IntakeSubsystem. */
  public Intake() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void intakeRun(){
    intaker.set(IntakeConstants.intakeSpeed);
    downTransporter.set(IntakeConstants.transportSpeed);

  }

  public void intakeStop(){
    intaker.set(0);
    downTransporter.set(0);
  }

  public void intakeReverse() {
    intaker.set(-IntakeConstants.intakeSpeed);
    downTransporter.set(-IntakeConstants.transportSpeed);
  }
}
