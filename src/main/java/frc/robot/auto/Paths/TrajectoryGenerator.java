package frc.robot.auto.Paths;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.MecanumControllerCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drive;

public class TrajectoryGenerator {
    Drive m_robotDrive;

    public TrajectoryGenerator(Drive drive) {
        m_robotDrive = drive;
    }

    public MecanumControllerCommand generate(
        String pathName,
        PIDController xController,
        PIDController yController,
        ProfiledPIDController thetaController
    ) {
        
        PathPlannerTrajectory trajectory = PathPlanner.loadPath(pathName, DriveConstants.kMaxVelocityMetersPerSecond, DriveConstants.kMaxAccelerationMetersPerSecondSquared);
        PathPlannerState endState = (PathPlannerState) trajectory.getEndState();

        return new MecanumControllerCommand(
            trajectory, 
            m_robotDrive::getMecanumPose,
            DriveConstants.kFeedforward, 
            DriveConstants.kMecanumDriveKinematics,
            xController, 
            yController, 
            thetaController, 
            DriveConstants.kMaxVelocityMetersPerSecond, 
            new PIDController(DriveConstants.kPDriveVel, 0, DriveConstants.kDDriveVel),
            new PIDController(DriveConstants.kPDriveVel, 0, DriveConstants.kDDriveVel),
            new PIDController(DriveConstants.kPDriveVel, 0, DriveConstants.kDDriveVel),
            new PIDController(DriveConstants.kPDriveVel, 0, DriveConstants.kDDriveVel),
            m_robotDrive::getCurrentMecanumWheelSpeeds,
            m_robotDrive::setDriveMotorControllersVolts,
            m_robotDrive);
        }
    
    public MecanumControllerCommand generateRotationalPrimary(
        String pathName,
        ProfiledPIDController thetaController
    ) {
        
        PathPlannerTrajectory trajectory = PathPlanner.loadPath(pathName, DriveConstants.kMaxVelocityMetersPerSecond, DriveConstants.kMaxAccelerationMetersPerSecondSquared);
        PathPlannerState endState = (PathPlannerState) trajectory.getEndState();

        return new MecanumControllerCommand(
            trajectory, 
            m_robotDrive::getMecanumPose,
            DriveConstants.kFeedforward, 
            DriveConstants.kMecanumDriveKinematics,
            DriveConstants.idealXController,
            DriveConstants.idealYController, 
            thetaController, 
            DriveConstants.kMaxVelocityMetersPerSecond, 
            new PIDController(DriveConstants.kPDriveVel, 0, DriveConstants.kDDriveVel),
            new PIDController(DriveConstants.kPDriveVel, 0, DriveConstants.kDDriveVel),
            new PIDController(DriveConstants.kPDriveVel, 0, DriveConstants.kDDriveVel),
            new PIDController(DriveConstants.kPDriveVel, 0, DriveConstants.kDDriveVel),
            m_robotDrive::getCurrentMecanumWheelSpeeds,
            m_robotDrive::setDriveMotorControllersVolts,
            m_robotDrive);
        }
    
    public MecanumControllerCommand generateTranslationalPrimary(
        String pathName,
        PIDController xController,
        PIDController yController
    ) {
        
        PathPlannerTrajectory trajectory = PathPlanner.loadPath(pathName, DriveConstants.kMaxVelocityMetersPerSecond, DriveConstants.kMaxAccelerationMetersPerSecondSquared);
        PathPlannerState endState = (PathPlannerState) trajectory.getEndState();

        return new MecanumControllerCommand(
            trajectory, 
            m_robotDrive::getMecanumPose,
            DriveConstants.kFeedforward, 
            DriveConstants.kMecanumDriveKinematics,
            xController,
            yController, 
            DriveConstants.idealThetaController, 
            DriveConstants.kMaxVelocityMetersPerSecond, 
            new PIDController(DriveConstants.kPDriveVel, 0, DriveConstants.kDDriveVel),
            new PIDController(DriveConstants.kPDriveVel, 0, DriveConstants.kDDriveVel),
            new PIDController(DriveConstants.kPDriveVel, 0, DriveConstants.kDDriveVel),
            new PIDController(DriveConstants.kPDriveVel, 0, DriveConstants.kDDriveVel),
            m_robotDrive::getCurrentMecanumWheelSpeeds,
            m_robotDrive::setDriveMotorControllersVolts,
            m_robotDrive);
        }  
}
