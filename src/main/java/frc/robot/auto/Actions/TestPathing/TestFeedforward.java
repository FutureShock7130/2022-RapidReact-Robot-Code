package frc.robot.auto.Actions.TestPathing;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drive;

public class TestFeedforward extends CommandBase {
    private final Drive m_drive;
    WPI_TalonFX motor;
    Timer timer = new Timer();

    public TestFeedforward(Drive m_robotDrive) {
        m_drive = m_robotDrive;
        addRequirements(m_robotDrive);
    }

    @Override
    public void initialize() {
        timer.start();
    }

    @Override
    public void execute() {
        m_drive.feedForwardTestDrive(12);
        SmartDashboard.putNumber("Target Motor Velocity", m_drive.getLinearWheelSpeeds());
        System.out.println(m_drive.getLinearWheelSpeeds());
    }

    public void end() { 
        m_drive.feedForwardTestDrive(0);
        timer.reset();
    }

    @Override
    public boolean isFinished() {
        double targetPulses = 1 / DriveConstants.kEncoderDistancePerPulse ;
        if (timer.get() > 3.5) {
            m_drive.feedForwardTestDrive(0);
            timer.reset();
            return true;
        }
        return false;
    }

    public void interrupted() {
        m_drive.feedForwardTestDrive(0);
        timer.reset();
    }
}
