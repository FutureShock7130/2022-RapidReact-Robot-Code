package frc.robot.commands.Superstructure;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SuperstructureConstants;
import frc.robot.subsystems.Superstructure;

public class SwingForward extends CommandBase{
    Superstructure m_SuperStructure;

    public SwingForward(Superstructure superStructure){
        m_SuperStructure = superStructure;
        addRequirements(superStructure);
    }

    public void initialize(){}

    public void execute(){
        if(!m_SuperStructure.atLimit()){
            m_SuperStructure.liftSwingRun(-SuperstructureConstants.swingSpeed);
        } else {
            m_SuperStructure.liftSwingStop();
        }
        System.out.println("lift swing");
        
    }

    public boolean isFinished(){
        return false;
    }

    public void end(boolean interrupted){
        m_SuperStructure.liftSwingStop();
    }

    public void interrupted(){
        m_SuperStructure.liftSwingStop();
    }

}
