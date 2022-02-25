package frc.robot.commands.Superstructure;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SuperstructureConstants;
import frc.robot.subsystems.Superstructure;

public class HangerGrab extends CommandBase {

    Superstructure m_Superstructure;

    double initTime = 0;

    public HangerGrab(Superstructure superStructure) {
        m_Superstructure = superStructure;
        addRequirements(superStructure);
    }

    public void initialize() {
        initTime = Timer.getFPGATimestamp();
    }

    public void execute() {
        while (Timer.getFPGATimestamp() < initTime + 1) {
            m_Superstructure.liftHangerRun(SuperstructureConstants.hangerSpeed);
        }
        initTime = Timer.getFPGATimestamp();
        while (Timer.getFPGATimestamp() < initTime + 1) {
            m_Superstructure.liftHangerRun(-SuperstructureConstants.hangerSpeed);
        }
        m_Superstructure.liftHangerStop();
    }

    public boolean isFinished() {
        return true;
    }

    public void end(boolean interrupted) {
        m_Superstructure.liftHangerStop();
    }
}
