package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TransporterConstants;

public class Transporter extends SubsystemBase {
  
  private final WPI_TalonSRX topTransporter = new WPI_TalonSRX(TransporterConstants.kTopTransporterID);
  
  // Creates a new TransporterSubsystem.
  public Transporter() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void transportRun(){
    topTransporter.set(TransporterConstants.transportSpeed);
  }

  public void transportEject(){
    topTransporter.set(-TransporterConstants.transportSpeed);
  }

  public void transportStop(){
    topTransporter.set(0);
  }
}
