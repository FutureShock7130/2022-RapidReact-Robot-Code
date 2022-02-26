package frc.robot.commands.Superstructure;

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

        if (LHangerPosition < SuperstructureConstants.HangerMaxPosition - 50
                || RHangerPosition < SuperstructureConstants.HangerMaxPosition - 50) {
            end(true);
        }
    }

    public void execute() {
        HangerDown hangerDown = new HangerDown(m_SuperStructure);
        HangerUp hangerUp = new HangerUp(m_SuperStructure);
        SwingBack swingBack = new SwingBack(m_SuperStructure);
        SwingForward swingForward = new SwingForward(m_SuperStructure);

        for (int i=0; i <= 3; i++) {
            hangerDown.schedule();
            while ((m_SuperStructure.getLHangerPosition() + m_SuperStructure.getRHangerPosition()) / 2 > 20) {
            }
            hangerDown.cancel();
            swingBack.schedule();
            while (m_SuperStructure.getSwingPosition() < SuperstructureConstants.SwingMaxPosition - 50) {
            }
            swingBack.cancel();
            hangerUp.schedule();
            while ((m_SuperStructure.getLHangerPosition() + m_SuperStructure.getRHangerPosition())
                    / 2 < SuperstructureConstants.HangerMaxPosition - 50) {
            }
            hangerUp.cancel();
            swingForward.schedule();
            while (m_SuperStructure.getSwingPosition() > 50) {
            }
            swingForward.cancel();
            hangerUp.schedule();

            while ((m_SuperStructure.getLHangerPosition() + m_SuperStructure.getRHangerPosition())
                    / 2 < SuperstructureConstants.HangerMaxPosition - 50) {
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
