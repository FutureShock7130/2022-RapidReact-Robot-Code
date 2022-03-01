package frc.robot.commands.Drive;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drive;

public class TrapezoidProfileDrive extends CommandBase {
    private Drive drive;
    private Timer timer = new Timer();
    private double output;

    private final SimpleMotorFeedforward feedforward = DriveConstants.kFeedforward;
    private final TrapezoidProfile.Constraints mConstraints = new TrapezoidProfile.Constraints(
        DriveConstants.kMaxVelocityMetersPerSecond,
        DriveConstants.kMaxAccelerationMetersPerSecondSquared);
    private TrapezoidProfile profile;

    private double kDt = 0.02;

    public TrapezoidProfileDrive(double meters, Drive m_drive) {
        drive = m_drive;
        profile = new TrapezoidProfile(
            mConstraints,
            new TrapezoidProfile.State(meters, 0),
            new TrapezoidProfile.State(0, 0)
        );
        drive.resetEncoders();
        addRequirements(m_drive);
    }

    @Override
  public void initialize() {
      timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      TrapezoidProfile.State setpoint = profile.calculate(timer.get());
      output = drive.feedforwardPIDDrive(setpoint.position, setpoint.velocity, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.feedForwardTestDrive(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (output <= 0.05)
    drive.feedForwardTestDrive(0);
    return true;
  }

  public void interrupted(){
    drive.feedForwardTestDrive(0);
  }
}