package frc.robot.auto.Actions.TestPathing;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drive;

public class TestOneMeters extends CommandBase {
    private final Drive m_drive;
    WPI_TalonFX motor;

    public TestOneMeters(Drive m_robotDrive) {
        m_drive = m_robotDrive;
        addRequirements(m_robotDrive);
    }

    @Override
    public void initialize() {
        motor = m_drive.getMotor(1);
    }

    @Override
    public void execute() {
        m_drive.feedForwardTestDrive(12);
        SmartDashboard.putNumber("Target Motor Velocity", m_drive.getTargetWheelSpeed(motor));
        System.out.println(m_drive.getTargetWheelSpeed(motor));
    }

    public void end() { 
        m_drive.feedForwardTestDrive(0);
    }

    @Override
    public boolean isFinished() {
        double targetPulses = 4 / DriveConstants.kEncoderDistancePerPulse ;
        if (m_drive.getTargetEncoderPositions(motor) > targetPulses) {
            m_drive.feedForwardTestDrive(0);
            return true;
        }
        return false;
    }

    public void interrupted() {
        m_drive.feedForwardTestDrive(0);
    }
}
