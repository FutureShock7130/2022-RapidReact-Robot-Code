// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SuperstructureConstants;

public class Superstructure extends SubsystemBase {
  // Creates a new Superstructure. 
  private final WPI_TalonSRX leftSwing = new WPI_TalonSRX(SuperstructureConstants.kSwingLeftID);
  private final WPI_TalonSRX rightSwing = new WPI_TalonSRX(SuperstructureConstants.kSwingRightID);
  private final CANSparkMax leftHanger = new CANSparkMax(SuperstructureConstants.kHangerLeftID, MotorType.kBrushless);
  private final CANSparkMax rightHanger = new CANSparkMax(SuperstructureConstants.kHangerRightID, MotorType.kBrushless);
  
  // Creates a new Superstructure.
  public Superstructure() {
    rightSwing.follow(leftSwing);
    rightHanger.follow(leftHanger, true);
    leftSwing.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void liftSwingRun(double speed){
    leftSwing.set(speed);
  }
  
  public void liftHangerRun(double speed) {
    leftHanger.set(speed);
  }   

  public void liftSwingStop(){
    leftSwing.set(0);
  }

  public void liftHangerStop(){
    leftHanger.set(0);
  }




}