// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

<<<<<<< HEAD
import java.sql.Time;
=======
import com.ctre.phoenix.motorcontrol.NeutralMode;
>>>>>>> 3d5fa1b167a1c480ba8eb5ba715b9e842c2ebf91
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

<<<<<<< HEAD
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
=======
import edu.wpi.first.wpilibj.DigitalInput;
>>>>>>> 3d5fa1b167a1c480ba8eb5ba715b9e842c2ebf91
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SuperstructureConstants;
import frc.robot.statemachines.SuperstructureStateMachine;
import edu.wpi.first.wpilibj.Timer;

public class Superstructure extends SubsystemBase {
  // Creates a new Superstructure.
  private final WPI_TalonSRX leftSwing = new WPI_TalonSRX(SuperstructureConstants.kSwingLeftID);
  private final WPI_TalonSRX rightSwing = new WPI_TalonSRX(SuperstructureConstants.kSwingRightID);
  private final CANSparkMax leftHanger = new CANSparkMax(SuperstructureConstants.kHangerLeftID, MotorType.kBrushless);
  private final CANSparkMax rightHanger = new CANSparkMax(SuperstructureConstants.kHangerRightID, MotorType.kBrushless);
  
  private SuperstructureStateMachine m_StateMachine;

  Timer time = new Timer();

  private final DigitalInput LlimitSwitch = new DigitalInput(SuperstructureConstants.LlimitSwitch);
  private final DigitalInput RlimitSwitch = new DigitalInput(SuperstructureConstants.RlimitSwitch);

  private boolean leftAtLimit;
  private boolean rightAtLimit;

  // Creates a new Superstructure.
  public Superstructure() {
    rightSwing.follow(leftSwing);

    leftSwing.setInverted(true);
    m_StateMachine = new SuperstructureStateMachine();
    rightHanger.setInverted(true);

    rightHanger.setIdleMode(IdleMode.kBrake);
    leftHanger.setIdleMode(IdleMode.kBrake);
    rightSwing.setNeutralMode(NeutralMode.Brake);
    leftSwing.setNeutralMode(NeutralMode.Brake);

    leftAtLimit = !getLlimitSwitchCheck();
    rightAtLimit = !getRlimitSwitchCheck();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putString("Current Superstructure Recommends:", m_StateMachine.getRecommendedHangarState(time.get()));
    leftAtLimit = !getLlimitSwitchCheck();
    rightAtLimit = !getRlimitSwitchCheck();
  }

  public void liftSwingRun(double speed) {
    leftSwing.set(speed);
  }

  public void liftHangerRun(double Lspeed, double Rspeed) {
    leftHanger.set(Lspeed);
    rightHanger.set(Rspeed);
  }

  public void liftSwingStop() {
    leftSwing.set(0);
  }

  public void liftHangerStop() {
    leftHanger.set(0);
    rightHanger.set(0);
  }

  public void initStateMachine() {
      time.start();
  }
  public boolean getLlimitSwitchCheck() {
    return LlimitSwitch.get();
  }

  public boolean getRlimitSwitchCheck() {
    return RlimitSwitch.get();
  }

  public boolean atLimit(){
    if (leftAtLimit || rightAtLimit){
      return true;
    }
    return false;
  }

}