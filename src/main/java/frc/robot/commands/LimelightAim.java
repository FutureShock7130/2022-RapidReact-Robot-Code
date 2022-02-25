package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import frc.robot.vision.Limelight;
import frc.robot.subsystems.Turret;


public class LimelightAim extends CommandBase {

    private static final double kPangle = 0.005;
    private static final double kIangle = 0;
    private static final double kDangle = 0;
    private static final double timeDiff = 0.02;

    private double derivative;
    private double output;

    private double xError;
    private double lastError;
    private double integralSumX;
    
    private Turret turret;
    private Limelight limelight;

    public LimelightAim(Turret m_robotTurret, Limelight m_robotLimelight) {
        turret = m_robotTurret;
        limelight = m_robotLimelight;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(turret);
        addRequirements(limelight);
    }

    public void initialize() {

    }

    public void execute() {
        if (checkLimit() == true) {
            return;
        }
        xError = Units.degreesToRadians(limelight.getX());

        if (Math.abs(integralSumX) < 100) {
            integralSumX += xError;
        }

        derivative = (xError - lastError) / timeDiff;
        output = kPangle * xError + kIangle * integralSumX + kDangle * derivative;

        turret.spinnerRun(output);
        lastError = xError;

        SmartDashboard.putNumber("output",output);
    }

    public void end(boolean interrupted) {
        if (interrupted) {
            turret.spinnerStop();
        }
    }

    public boolean isFinished() {
        if (Math.abs(xError) < 0.1){
            return true;
        }

        integralSumX = 0;
        return false;
    }

    public boolean checkLimit() {    
        if (turret.getforwardLimitSwitchCheck() == false) {
            // if touched limit switch -> return true
            return true;
        }           
        else if(turret.getreverseLimitSwitchCheck() ==  false) {
            return true;
        }
        else {
            return false;
        }
    }
}
