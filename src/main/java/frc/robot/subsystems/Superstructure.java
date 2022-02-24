package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SuperstructureConstants;

public class Superstructure extends SubsystemBase {
  
  private final WPI_TalonSRX SwingLeft = new WPI_TalonSRX(SuperstructureConstants.kSwingLeftID);
  private final WPI_TalonSRX SwingRight = new WPI_TalonSRX(SuperstructureConstants.kSwingRightID);
  private final CANSparkMax hangerLeft = new CANSparkMax(SuperstructureConstants.kHangerLeftID, MotorType.kBrushless);
  private final CANSparkMax hangerRight = new CANSparkMax(SuperstructureConstants.kHangerRightID, MotorType.kBrushless);
  
  // Creates a new Superstructure.
  public void Superstructure() {
    SwingRight.follow(SwingLeft);
    hangerRight.follow(hangerLeft);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void swingRun() {
    SwingLeft.set(SuperstructureConstants.swingSpeed);
  }

  public void swingStop() {
    SwingLeft.set(0);
  }
  
  public void hangRun() {
    hangerLeft.set(SuperstructureConstants.hangerSpeed);
  }   

  public void hangStop() {
    hangerLeft.set(0);
  }


  
}
