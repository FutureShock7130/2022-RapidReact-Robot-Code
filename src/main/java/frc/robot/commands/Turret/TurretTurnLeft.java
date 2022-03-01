package frc.robot.commands.Turret;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Spinner;
import frc.robot.subsystems.Turret;


public class TurretTurnLeft extends CommandBase {

    private Spinner spinner;
    private double turnSpeed = 0;

    public TurretTurnLeft(Spinner m_robotSpinner, double speed) {
        spinner = m_robotSpinner;
        turnSpeed = speed;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(spinner);
    }

    public void initialize() {

    }

    public void execute() {
        // if (checkLimit() == true) {
        //     return;
        // }
        if (!spinner.atLeftLimit()) {
            spinner.spinnerRun(turnSpeed);
        }

        SmartDashboard.putNumber("spinnerLeft Speed:", turnSpeed);
        System.out.println("spinnerTurnLeft executing");
    }

    public void end(boolean interrupted) {
        spinner.spinnerRun(0);
    }

    public boolean isFinished() {
        if (spinner.atTurningLimit()) {
            spinner.spinnerRun(0);
            return true;
        }
        return false;
    }
}
