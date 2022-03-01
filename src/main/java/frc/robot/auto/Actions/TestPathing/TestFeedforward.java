package frc.robot.auto.actions.TestPathing;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drive;

public class TestFeedforward extends CommandBase {
    private final Drive m_drive;
    WPI_TalonFX motor;
    Timer timer = new Timer();

    private SimpleMotorFeedforward feedforward = DriveConstants.kFeedforward;

    double initialEncoderPos;
    double kDt = 0.02;
    double linearDisplacement;
    double linearVelocity;
    double linearAcceleration;

    double lastVelocity;

    String dataMessage;

    private final double kPercentageVoltage = 0.8;

    public TestFeedforward(Drive m_robotDrive) {
        m_drive = m_robotDrive;
        addRequirements(m_robotDrive);
    }

    @Override
    public void initialize() {
        timer.start();
        m_drive.resetEncoders();
        initialEncoderPos = m_drive.getLinearEncoderPosition();
    }

    @Override
    public void execute() {
        linearDisplacement = m_drive.getLinearEncoderPosition() - initialEncoderPos;
        linearVelocity = m_drive.getLinearWheelSpeeds();
        linearAcceleration = (linearVelocity - lastVelocity) / kDt;
        m_drive.feedForwardTestDrive(DriveConstants.kS);
        //m_drive.feedForwardTestDrive(12 * kPercentageVoltage);
        SmartDashboard.putNumber("Linear Acceleration", linearAcceleration);
        SmartDashboard.putNumber("Linear Velocity", linearAcceleration);
        SmartDashboard.putNumber("Linear Displacement" , linearDisplacement);
        lastVelocity = linearVelocity;

        String data = String.format("%f, %f, %f", linearDisplacement*DriveConstants.kEncoderDistancePerPulse/10, linearVelocity, linearAcceleration);
        System.out.println(data);
    }

    public void end() { 
        m_drive.feedForwardTestDrive(0);
        timer.reset();
    }

    @Override
    public boolean isFinished() {
        double targetPulses = 1 / DriveConstants.kEncoderDistancePerPulse ;
        if (timer.get() > 2) {
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
