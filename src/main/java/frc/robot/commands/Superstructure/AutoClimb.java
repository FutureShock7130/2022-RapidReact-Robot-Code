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

        if (LHangerPosition < SuperstructureConstants.HangerMaxPosition-50 || RHangerPosition < SuperstructureConstants.HangerMaxPosition-50){

        }
    }

    public void execute(){

    }

    public boolean isFinished(){
        return true;
    }

    public void end(boolean interrupted){
        m_SuperStructure.liftSwingStop();
        m_SuperStructure.liftHangerStop();
    }
}
