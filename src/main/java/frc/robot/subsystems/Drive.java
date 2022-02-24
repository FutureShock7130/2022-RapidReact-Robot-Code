package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive extends SubsystemBase {
    public Drive() 
    {

    }

    public double arcadeDrive() {
        return 1.0;
    }

    public double tankDrive() {
        return 2.0;
    }

    public double mecanumDrive() {
        return 3.0;
    }

    public Encoder getEncoders() {
        Encoder encoder = null;
        return encoder;
    }
}