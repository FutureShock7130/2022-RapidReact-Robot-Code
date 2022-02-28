package frc.robot.commands.Superstructure;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Superstructure;

public class SwingStop extends CommandBase{
    Superstructure m_SuperStructure;

    public SwingStop(Superstructure superStructure){
        m_SuperStructure = superStructure;
        addRequirements(superStructure);
    }

    public void initialize(){}

    public void execute(){
        m_SuperStructure.liftSwingStop();
        
    }

    public boolean isFinished(){
        return true;
    }

    public void end(boolean interrupted){
        m_SuperStructure.liftSwingStop();
    }

}
