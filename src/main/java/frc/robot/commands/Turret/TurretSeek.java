package frc.robot.commands.Turret;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.Turret;

public class TurretSeek extends CommandBase{
    
    Turret m_turret;

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tvEntry = table.getEntry("tv");
    
    double tv;

    double speed=TurretConstants.turretSpeedMultiplier/2;

    boolean targetFound = false;

    public TurretSeek(Turret m_robotTurret){
        m_turret = m_robotTurret;
        addRequirements(m_robotTurret);
    }

    public void initialize(){
        tv = tvEntry.getDouble(0.0);
    }

    public void execute(){
        tv = tvEntry.getDouble(0.0);
        if (tv == 0.0d){
            
            if (m_turret.atLeftLimit()) speed = TurretConstants.turretSpeedMultiplier/2;
            if (m_turret.atRightLimit()) speed = -TurretConstants.turretSpeedMultiplier/2;

            m_turret.spinnerRun(speed);

        } else {
            m_turret.spinnerRun(0);
            targetFound = true;
        }
        System.out.println(tv);
    }

    public boolean isFinished(){
        if(targetFound) return true;
        return false;
    }

    public void end(boolean interrupted){
        m_turret.spinnerRun(0);
    }
    
    public void interrupted(){
        m_turret.spinnerRun(0);
    }

}
