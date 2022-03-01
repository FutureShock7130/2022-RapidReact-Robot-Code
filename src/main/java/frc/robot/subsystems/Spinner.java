// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;

public class Spinner extends SubsystemBase {

  private final WPI_TalonSRX spinner = new WPI_TalonSRX(TurretConstants.kSpinnerID);
  private final DigitalInput forwardLimitSwitch = new DigitalInput(TurretConstants.forwardLimitSwitch);
  private final DigitalInput reverseLimitSwitch = new DigitalInput(TurretConstants.reverseLimitSwitch);

  private boolean leftAtLimit;
  private boolean rightAtLimit;

  /** Creates a new Spinner. */
  public Spinner() {
    leftAtLimit = !getforwardLimitSwitchCheck();
    rightAtLimit = !getreverseLimitSwitchCheck();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    SmartDashboard.putBoolean("Left Turning Limit", leftAtLimit);
    SmartDashboard.putBoolean("Right Turning Limit", rightAtLimit);
    leftAtLimit = !getforwardLimitSwitchCheck();
    rightAtLimit = !getreverseLimitSwitchCheck();
  }

  public void spinnerRun(double speed) {
    spinner.set(speed);
  }

  public void spinnerStop() {
    spinner.set(0);
  }

  public boolean getforwardLimitSwitchCheck() {
    return forwardLimitSwitch.get();
  }

  public boolean getreverseLimitSwitchCheck() {
    return reverseLimitSwitch.get();
  }

  public boolean atTurningLimit() {
    if (leftAtLimit || rightAtLimit) {
      return true;
    }
    return false;

  }

  public boolean atLeftLimit() {
    return leftAtLimit;
  }

  public boolean atRightLimit() {
    return rightAtLimit;
  }

}
