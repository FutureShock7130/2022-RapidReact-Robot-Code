package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;

public class Turret extends SubsystemBase {

    private final CANSparkMax masterFlyWheel = new CANSparkMax(TurretConstants.kMasterFlyWheelID, MotorType.kBrushless);
    private final CANSparkMax slaveFlyWheel = new CANSparkMax(TurretConstants.kSlaveFlyWheelID, MotorType.kBrushless); 
    private final WPI_TalonSRX spinner = new WPI_TalonSRX(TurretConstants.kSpinnerID);
    private final DigitalInput forwardLimitSwitch = new DigitalInput(TurretConstants.forwardLimitSwitch);
    private final DigitalInput reverseLimitSwitch = new DigitalInput(TurretConstants.reverseLimitSwitch);

    private boolean leftAtLimit;
    private boolean rightAtLimit;

    /** Creates a new TurretSubystem. */
    public Turret() {
        masterFlyWheel.setInverted(true);
        slaveFlyWheel.follow(masterFlyWheel, true);
        leftAtLimit = !getforwardLimitSwitchCheck();
        rightAtLimit = !getreverseLimitSwitchCheck();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Flywheel Speed", getFlyWheelsVelocity());
        SmartDashboard.putBoolean("Left Turning Limit", leftAtLimit);
        SmartDashboard.putBoolean("Right Turning Limit", rightAtLimit);
        leftAtLimit = !getforwardLimitSwitchCheck();
        rightAtLimit = !getreverseLimitSwitchCheck();
    }

    public void flywheelsRun(double speed) {
        masterFlyWheel.set(speed);
    }

    public void spinnerRun(double speed) {
        spinner.set(speed);
    }

    public void flywheelsStop() {
        masterFlyWheel.set(0);
    }

    public void spinnerStop() {
        spinner.set(0);
    }

    public double getFlyWheelsVelocity() {
        return (masterFlyWheel.getEncoder().getVelocity()+slaveFlyWheel.getEncoder().getVelocity())/2;
    }

    public boolean getforwardLimitSwitchCheck() {
        return forwardLimitSwitch.get();
    }

    public boolean getreverseLimitSwitchCheck() {
        return reverseLimitSwitch.get();
    }

    public boolean atTurningLimit()
    {
        if (leftAtLimit || rightAtLimit){
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