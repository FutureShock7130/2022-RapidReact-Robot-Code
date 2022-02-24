package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;

public class Turret extends SubsystemBase {

    private final CANSparkMax masterFlyWheel = new CANSparkMax(TurretConstants.kMasterFlyWheelID, MotorType.kBrushless);
    private final CANSparkMax slaveFlyWheel = new CANSparkMax(TurretConstants.kSlaveFlyWheelID, MotorType.kBrushless); 
    private final WPI_TalonSRX spinner = new WPI_TalonSRX(TurretConstants.kSpinnerID);
    private final DigitalInput forwardLimitSwitch = new DigitalInput(TurretConstants.forwardLimitSwitch);
    private final DigitalInput reverseLimitSwitch = new DigitalInput(TurretConstants.reverseLimitSwitch);


    private int output;
    /** Creates a new TurretSubystem. */
    public Turret() {
        slaveFlyWheel.follow(masterFlyWheel);
    }

    

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
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
        return masterFlyWheel.getEncoder().getVelocity();
    }
    public int checkLimit()
    {
        
        if (forwardLimitSwitch.get()){
            output = 1;
            return output;
        }           
        else if(reverseLimitSwitch.get()){
            output = 1;
            return output;
        }
        else{
            output = 0;
            return output;
        }
            
    }
}