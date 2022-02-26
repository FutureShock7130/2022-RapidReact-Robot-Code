package frc.robot.commands.Superstructure;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SuperstructureConstants;
import frc.robot.subsystems.Superstructure;

public class AutoClimb extends CommandBase {

    private Superstructure m_SuperStructure;

    private double LHangerPosition;
    private double RHangerPosition;

    public AutoClimb(Superstructure m_Superstructure) {
        m_SuperStructure = m_Superstructure;
    }

    public void initialize() {
        // Inits after the hanger already touches

        // Check hanger current position
        LHangerPosition = m_SuperStructure.getLHangerPosition();
        RHangerPosition = m_SuperStructure.getRHangerPosition();

        if (LHangerPosition < SuperstructureConstants.HangerMaxPosition - 2
                || RHangerPosition < SuperstructureConstants.HangerMaxPosition - 2) {
            end(true);
        }
    }

    public void execute() {
        HangerDown hangerDown = new HangerDown(m_SuperStructure);
        HangerUp hangerUp = new HangerUp(m_SuperStructure);
        SwingBack swingBack = new SwingBack(m_SuperStructure);
        SwingForward swingForward = new SwingForward(m_SuperStructure);

        double lastTime;

        for (int i=0; i <= 3; i++) {
            hangerDown.schedule();
            while ((m_SuperStructure.getLHangerPosition() + m_SuperStructure.getRHangerPosition()) / 2 > SuperstructureConstants.HangerMinPosition+3) {
            }
            hangerDown.cancel();
            lastTime = Timer.getFPGATimestamp();
            swingBack.schedule();
            while (Timer.getFPGATimestamp()< lastTime+0.5) {
            }
            swingBack.cancel();
            hangerUp.schedule();
            while ((m_SuperStructure.getLHangerPosition() + m_SuperStructure.getRHangerPosition())
                    / 2 < SuperstructureConstants.HangerMaxPosition - 3) {
            }
            hangerUp.cancel();
            lastTime = Timer.getFPGATimestamp();
            swingForward.schedule();
            while (Timer.getFPGATimestamp() < lastTime+0.5) {
            }
            swingForward.cancel();
            hangerUp.schedule();

            while ((m_SuperStructure.getLHangerPosition() + m_SuperStructure.getRHangerPosition())
                    / 2 < SuperstructureConstants.HangerMaxPosition - 3) {
            }
            hangerUp.cancel();
            // Start of next level
        }
    }

    public boolean isFinished() {
        return true;
    }

    public void end(boolean interrupted) {
        m_SuperStructure.liftSwingStop();
        m_SuperStructure.liftHangerStop();
    }
}
