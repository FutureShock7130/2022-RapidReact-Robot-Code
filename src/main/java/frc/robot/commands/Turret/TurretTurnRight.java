package frc.robot.commands.Turret;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import frc.robot.vision.Limelight;
import frc.robot.subsystems.Turret;


public class TurretTurnRight extends CommandBase {
    private Turret turret;
    private double turnSpeed = 0;

    public TurretTurnRight(Turret m_robotTurret, double speed) {
        turret = m_robotTurret;
        turnSpeed = speed;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(turret);
    }

    public void initialize() {

    }

    public void execute() {
        // if (checkLimit() == true) {
        //     return;
        // }
        if (!turret.atRightLimit()) {
            turret.spinnerRun(-turnSpeed);
        }

        SmartDashboard.putNumber("TurretLeft Speed:", turnSpeed);
        System.out.println("TurretTurnLeft executing");
    }

    public void end(boolean interrupted) {
        turret.spinnerRun(0);
    }

    public boolean isFinished() {
        if (turret.atTurningLimit()) {
            turret.spinnerRun(0);
            return true;
        }
        return false;
    }
}
