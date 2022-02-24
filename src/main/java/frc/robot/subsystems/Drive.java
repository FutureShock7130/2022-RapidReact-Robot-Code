package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive extends SubsystemBase {
    
    private TalonSRX frontLeft = new TalonSRX();
    private TalonSRX rearLeft = new TalonSRX();
    private TalonSRX frontRight = new TalonSRX();
    private TalonSRX rearRight = new TalonSRX();

    public Drive() {

    }
}