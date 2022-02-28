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

        addRequirements(m_Superstructure);
    }

    public void initialize() {
        // Inits after the hanger already touches

        // Check hanger current position
        LHangerPosition = m_SuperStructure.getLHangerPosition();
        RHangerPosition = m_SuperStructure.getRHangerPosition();

        if (LHangerPosition < SuperstructureConstants.HangerMaxPosition - 2
                || RHangerPosition < SuperstructureConstants.HangerMaxPosition - 2) {
            end(true);
            System.out.println("not accepted");
        }
    }

    public void execute() {

        double lastTime;
        double[] maxPosition; // [left, right]
        double[] minPosition = { 0.0, 0.0 };

        for (int i = 0; i < 3; i++) {

            lastTime = Timer.getFPGATimestamp();
            maxPosition = new double[] { m_SuperStructure.getLHangerPosition(), m_SuperStructure.getRHangerPosition() };
            if (i > 0) {
                maxPosition[0] += 5;
                maxPosition[1] += 5;
            }
            System.out.println("Max 1" + maxPosition[0] + " " + maxPosition[1]);
            System.out.println("Min 1" + minPosition[0] + " " + minPosition[1]);
            while (m_SuperStructure.getLHangerPosition() > minPosition[0] + 1.5
                    || m_SuperStructure.getLHangerPosition() > minPosition[0] + 1.5) {
                double leftSpeed = m_SuperStructure.getLHangerPosition() > minPosition[0] + 1.5
                        ? -SuperstructureConstants.hangerSpeed
                        : 0;
                double rightSpeed = m_SuperStructure.getRHangerPosition() > minPosition[1] + 1.5
                        ? -SuperstructureConstants.hangerSpeed
                        : 0;
                m_SuperStructure.liftHangerRun(leftSpeed, rightSpeed);
            }
            m_SuperStructure.liftHangerStop();
            m_SuperStructure.resetEncoder(m_SuperStructure.getLHangerPosition() - 1.6,
                    m_SuperStructure.getRHangerPosition() - 1.6);
            minPosition = new double[] { m_SuperStructure.getLHangerPosition() - 1.5,
                    m_SuperStructure.getRHangerPosition() - 1.5 };

            lastTime = Timer.getFPGATimestamp();
            while (Timer.getFPGATimestamp() < lastTime + 0.5) {
                m_SuperStructure.liftSwingRun(SuperstructureConstants.swingSpeed);
            }
            m_SuperStructure.liftSwingStop();

            while (m_SuperStructure.getLHangerPosition() < maxPosition[0] - 5
                    || m_SuperStructure.getLHangerPosition() < maxPosition[0] - 5) {
                double leftSpeed = m_SuperStructure.getLHangerPosition() < maxPosition[0] - 5
                        ? SuperstructureConstants.hangerSpeed
                        : 0;
                double rightSpeed = m_SuperStructure.getRHangerPosition() < maxPosition[1] - 5
                        ? SuperstructureConstants.hangerSpeed
                        : 0;
                m_SuperStructure.liftHangerRun(leftSpeed, rightSpeed);
            }
            maxPosition = new double[] { m_SuperStructure.getLHangerPosition(), m_SuperStructure.getRHangerPosition() };

            lastTime = Timer.getFPGATimestamp();
            while (Timer.getFPGATimestamp() < lastTime + 1) {
                m_SuperStructure.liftSwingRun(-SuperstructureConstants.swingSpeed);
            }
            m_SuperStructure.liftSwingStop();
            System.out.println("Max 2" + maxPosition[0] + " " + maxPosition[1]);
            System.out.println("Min 2" + minPosition[0] + " " + minPosition[1]);
        }

    }

    public boolean isFinished() {
        return true;
    }

    public void end(boolean interrupted) {
        m_SuperStructure.liftSwingStop();
        m_SuperStructure.liftHangerStop();
    }

    public void interrupted() {
        m_SuperStructure.liftSwingStop();
        m_SuperStructure.liftHangerStop();
    }
}
