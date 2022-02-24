package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Limelight;


public class LimelightAim extends CommandBase {

    private static final double kPangle = 1;
    private static final double kIangle = 0.005;
    private static final double kDangle = 0;
    private static final double timeDiff = 0.02;
    
    private static final double kPdist = 0.25;
    private static final double kIdist = 0;
    private static final double kDdist = 0;

    private double distError;
    private double integralSumDist;
    private double lastDistError;

    private double xError;
    private double integralSumX;
    private double lastError;

    private Drive m_drive;
    private Limelight m_limelight;

    public LimelightAim(Drive driveSubsystem, Limelight limelight) {
        m_drive = driveSubsystem;
        m_limelight = limelight;
        addRequirements(driveSubsystem);
        addRequirements(limelight);
    }

    public void initialize() {
        xError = m_limelight.getX()*Math.PI/180;
        distError = m_limelight.getDis();
        System.out.println("Executing Limelight Aim");
    }

    public void execute() {
        xError = m_limelight.getX()*Math.PI/180;
        distError = m_limelight.getDis();
        if (Math.abs(integralSumX) < 100) {
            integralSumX += xError;
        }
        if (Math.abs(integralSumDist) < 1000){
            integralSumDist += distError;
        }

        double distDerivative = (distError - lastDistError) / timeDiff;
        double derivative = (xError - lastError) / timeDiff;

        double output = kPangle * xError + kIangle * integralSumX + kDangle * derivative;
        double distOutput = kPdist * distError + kIdist * integralSumDist + kDdist * distDerivative;

        m_drive.drive(distOutput,0, output,false);

        lastError = xError;
        lastDistError = distError;
        SmartDashboard.putNumber("output",output);
        SmartDashboard.putNumber("distError", distError);
    }

    public void end(boolean interrupted) {
        if (interrupted) {
            m_drive.drive(0, 0, 0, false);
        }
    }

    public boolean isFinished() {
        if (Math.abs(xError) < 0.1 && Math.abs(distError)<3){
            return true;
        }
        integralSumX = 0;
        integralSumDist = 0;
        return false;
    }
}
