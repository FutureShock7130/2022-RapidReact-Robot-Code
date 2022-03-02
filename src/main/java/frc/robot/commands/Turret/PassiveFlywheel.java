package frc.robot.commands.Turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

public class PassiveFlywheel extends CommandBase {
    Turret m_turret;
    public PassiveFlywheel(Turret turret) {
        m_turret = turret;
        addRequirements(m_turret);
    }
    
    public void initialize() { }

    public void execute() {
        m_turret.flywheelsRun(-0.001);
    }

    public void end() { }

    @Override
    public boolean isFinished() {
        return true;
    }
}
