package frc.robot.commands.Turret;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.vision.Limelight;
import frc.robot.subsystems.Spinner;
import frc.robot.subsystems.Turret;


public class LimelightAim extends CommandBase {

    private static final double kP = 0.6;
    private static final double kI = 0.001;

    private double output;

    private double xError;
    private double integralSumX;

    private Limelight limelight;
    private Spinner spinner;

    public LimelightAim(Limelight m_robotLimelight, Spinner m_robotSpinner) {
        limelight = m_robotLimelight;
        spinner = m_robotSpinner;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(limelight);
        addRequirements(spinner);
    }

    public void initialize() {
        System.out.println("LimelightAim executing");
    }

    public void execute() {
        // if (checkLimit() == true) {
        //     return;
        // }
        
        xError = Units.degreesToRadians(limelight.getX());

        if (Math.abs(integralSumX) < 100) {
            integralSumX += xError;
        }

        output = kP * xError + kI * integralSumX;

        if (!spinner.atTurningLimit()) {
            spinner.spinnerRun(-output);  
        }

        SmartDashboard.putNumber("Limelight Auto Aim Output", output);
    }

    public void end(boolean interrupted) {
        spinner.spinnerRun(0);
    }

    public boolean isFinished() {
        if (Math.abs(limelight.getX()) < 0.1){
            spinner.spinnerRun(0);
            return true;
        }
        integralSumX = 0;
        return false;
    }

    public void interrupted() {
        spinner.spinnerRun(0);
    }
}
