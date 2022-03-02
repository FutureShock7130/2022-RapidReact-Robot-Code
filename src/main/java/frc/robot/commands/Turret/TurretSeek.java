package frc.robot.commands.Turret;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.Spinner;
import frc.robot.subsystems.Turret;
import frc.robot.vision.Limelight;

public class TurretSeek extends CommandBase {

    private Spinner spinner;
    private Limelight vision;

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tvEntry = table.getEntry("tv");

    double tv;
    double speed = TurretConstants.turretSpeedMultiplier / 2;

    public TurretSeek(Spinner m_robotSpinner, Limelight limelight) {
        spinner = m_robotSpinner;
        vision = limelight;

        addRequirements(m_robotSpinner);
    }

    public void initialize() {
    }

    public void execute() {
        tv = tvEntry.getDouble(0.0);
        if (tv == 0.0d || vision.getTargetStatus() == false) {
            if (spinner.atLeftLimit())
                speed = TurretConstants.turretSpeedMultiplier / 2;
            if (spinner.atRightLimit())
                speed = -TurretConstants.turretSpeedMultiplier / 2;
            spinner.spinnerRun(speed);
        } else {
            spinner.spinnerRun(0);
        }
    }

    public boolean isFinished() {
        if (vision.getTargetStatus() == true) {
            spinner.spinnerRun(0);
            return true;
        }
        return false;
    }

    public void end(boolean interrupted) {
        spinner.spinnerRun(0);
    }

    public void interrupted() {
        spinner.spinnerRun(0);
    }

}
