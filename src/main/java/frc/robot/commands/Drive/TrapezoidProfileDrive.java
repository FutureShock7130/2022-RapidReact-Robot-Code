package frc.robot.commands.Drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    private double linearDisplacement;
    private double linearVelocity;
    private double linearAcceleration;
    private double lastVelocity;

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

      linearDisplacement = drive.getLinearEncoderPosition();
      linearVelocity = drive.getLinearWheelSpeeds();
      linearAcceleration = (linearVelocity - lastVelocity) / kDt;
      drive.feedForwardTestDrive(DriveConstants.kS);
      //m_drive.feedForwardTestDrive(12 * kPercentageVoltage);
      lastVelocity = linearVelocity;
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

  private double kP = 0.0001;
  private double kI = 0;
  private double kD = 0;
  private PIDController controllerFL = new PIDController(kP, kI, kD);
  private PIDController controllerRL = new PIDController(kP, kI, kD);
  private PIDController controllerFR = new PIDController(kP, kI, kD);
  private PIDController controllerRR = new PIDController(kP, kI, kD);

  private double kPVel = DriveConstants.kPDriveVel;

  // Feedforward control defined above as a constant

  // Feedforward Drive using the WPI Controllers
  public double feedforwardPIDDrive(double targetPos, double velocity, double acceleration) {
    // Note that Velocity is in m/s
    targetPos /= DriveConstants.kEncoderDistancePerPulse;
    double kF = feedforward.calculate(velocity, acceleration);
    double velError = velocity - drive.getLinearWheelSpeeds();
    drive.motorFL.setVoltage(
      velError * kPVel +
      controllerFL.calculate(drive.motorFL.getSelectedSensorPosition(), targetPos) + kF);
    drive.motorRL.setVoltage(
      velError * kPVel +
      controllerRL.calculate(drive.motorRL.getSelectedSensorPosition(), targetPos) + kF);
    drive.motorFR.setVoltage(
      velError * kPVel +
      controllerFR.calculate(drive.motorFR.getSelectedSensorPosition(), targetPos) + kF);
    drive.motorRR.setVoltage(
      velError * kPVel +
      controllerRR.calculate(drive.motorRR.getSelectedSensorPosition(), targetPos) + kF);
    return controllerFL.calculate(targetPos, drive.motorFL.getSelectedSensorPosition()) + kF;
  }
}