package frc.robot.auto.Actions;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Turret.LimelightAim;
import frc.robot.commands.Turret.TurretSeek;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Spinner;
import frc.robot.vision.Limelight;

public class AutoAim {

    private Drive m_robotDrive;
    private Spinner m_Spinner;
    private Limelight m_vision;

    public AutoAim(Drive robotDrive, Spinner robotSpinner, Limelight vision){
        m_robotDrive = robotDrive;
        m_Spinner = robotSpinner;
        m_vision = vision;
    }
    
    public Command getCommand(){
        SequentialCommandGroup command = new SequentialCommandGroup(
            //new AbsoluteAim(m_robotDrive),
            new TurretSeek(m_Spinner, m_vision),
            new LimelightAim(m_vision, m_Spinner)
        );
        return command;
    }

}
