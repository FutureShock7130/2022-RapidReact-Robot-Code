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

    /** Creates a new TurretSubystem. */
    public Turret() {
        masterFlyWheel.setInverted(true);
        slaveFlyWheel.setInverted(false);
        slaveFlyWheel.follow(masterFlyWheel, true);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Flywheel Velocity (Periodic)", getFlyWheelsVelocity());
    }

    public void flywheelsRun(double speed) {
        masterFlyWheel.set(speed);
        slaveFlyWheel.set(speed);
    }


    public void flywheelsStop() {
        masterFlyWheel.set(0);
        slaveFlyWheel.set(0);
    }

    public double getFlyWheelsVelocity() {
        return (masterFlyWheel.getEncoder().getVelocity() + slaveFlyWheel.getEncoder().getVelocity()) / 2;
    }
}