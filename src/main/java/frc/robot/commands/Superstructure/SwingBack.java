package frc.robot.commands.Superstructure;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SuperstructureConstants;
import frc.robot.subsystems.Superstructure;

public class SwingBack extends CommandBase {
    Superstructure m_SuperStructure;
    double time;

    public SwingBack(Superstructure superStructure) {
        m_SuperStructure = superStructure;
        addRequirements(superStructure);
    }

    public void initialize() {
        time = Timer.getFPGATimestamp();
    }

    public void execute() {
        while (Timer.getFPGATimestamp() < time + 0.5) {
            m_SuperStructure.liftSwingRun(SuperstructureConstants.swingSpeed);
        }
        m_SuperStructure.liftSwingStop();
    }

    public boolean isFinished() {
        return true;
    }

    public void end(boolean interrupted) {
        m_SuperStructure.liftSwingStop();
    }

    public void interrupted() {
        m_SuperStructure.liftSwingStop();
    }

}
