package frc.robot.auto.Actions.TestPathing;

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
    double kDt = 0.1;
    double linearDisplacement;
    double linearVelocity;
    double linearAcceleration;

    double maxVelocity = 0;
    double maxAcc = 0;

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
        m_drive.feedForwardTestDrive(11.4);
        //m_drive.feedForwardTestDrive(12 * kPercentageVoltage);
        SmartDashboard.putNumber("Linear Acceleration", linearAcceleration);
        SmartDashboard.putNumber("Linear Velocity", linearAcceleration);
        SmartDashboard.putNumber("Linear Displacement" , linearDisplacement);
        lastVelocity = linearVelocity;

        if (linearVelocity > maxVelocity) {
            maxVelocity = linearVelocity;
        }

        if (linearAcceleration > maxAcc) {
            maxAcc = linearAcceleration;
        }

        String data = String.format("%f, %f", timer.get(), linearVelocity);
        System.out.println(data);
    }

    public void end() { 
        m_drive.feedForwardTestDrive(0);
        timer.reset();

        String data = String.format("%f, %f, %f", maxVelocity, maxAcc);
        System.out.println(data);
    }

    @Override
    public boolean isFinished() {
        double targetPulses = 1 / DriveConstants.kEncoderDistancePerPulse ;
        if (timer.get() > 3) {
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
