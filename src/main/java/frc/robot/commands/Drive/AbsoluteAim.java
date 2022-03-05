package frc.robot.commands.Drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;

public class AbsoluteAim extends CommandBase {
        // This command is not going to be used in the competition
        // Sorry Aaron!!

        // This command is used to allow the robot to aim at a specific coordinate at
        // any point on the field.
        // Future update will make its target adjustable without changing the class
        // itself.

        Drive m_robotDrive;

        Translation2d initTranslation;
        Rotation2d initRotation;
        Rotation2d changeAngle;
        Rotation2d finalRotation;

        Rotation2d currentRotation;
        Rotation2d lastRotation;

        double targetX = 0;
        double targetY = -1;

        private static final double kP = 0.027;
        private static final double kI = 0.00125;
        private static final double kD = 0.002;
        private static final double timeDiff = 0.02;

        private double rError;
        private double integralSumR;
        private double lastError;

        double angle;
        private  boolean relative;

        public AbsoluteAim(Drive robotDrive, boolean relative, double changeAngle ) {
                m_robotDrive = robotDrive;
                angle = changeAngle;
                this.relative = relative;
                addRequirements(robotDrive);
        }

        @Override
        public void initialize() {
                System.out.println("Executing Absolute Aim");

                initTranslation = m_robotDrive.getPose().getTranslation();
                initRotation = m_robotDrive.getPose().getRotation();
                // get translation2d object and rotation2d object from the Pose2d object from
                // the DriveSubsysten.
               
                // current coordinate
                double changeCalc = angle / 180 * Math.PI;
                // Calculate the angle difference between the two coordinate (if the current
                // rotation(angle) is 0)
                changeAngle = new Rotation2d(changeCalc);
                // Add the current angle into the rotation. Think of it as first correcting the
                // angle of the robot to 0,
                // then turn the actual angle difference between the two coordinates.
                changeAngle = changeAngle.getRadians() > Math.PI ? changeAngle.minus(new Rotation2d(2 * Math.PI))
                                : changeAngle;
                // if the angle is greater than Pi, the angle is too big (larger than half the
                // circle)
                // and it's better to rotate the other way around. So I used this inline if
                // statement (?: statement) to assign the angle
                // if the angle is greater than Pi, than it should turn the angle-2pi (draw a
                // diagram to help you think about it)
                finalRotation = initRotation.rotateBy(changeAngle);
                // Calculate the final angle that the robot should be at, which will be used to
                // determine when the rotation should stop
                integralSumR = 0;
        }

        @Override
        public void execute() {
                currentRotation = m_robotDrive.getPose().getRotation();
                rError = finalRotation.minus(currentRotation).getRadians();

                if (Math.abs(integralSumR) < 50) {
                        integralSumR += rError;
                }

                double derivative = (rError - lastError) / timeDiff;

                double output = kP * rError + kI * integralSumR + kD * derivative;

                m_robotDrive.drive(0, 0, output, false);

                lastError = rError;

                SmartDashboard.putNumber("output", output);
        }

        public void end(boolean isInterrupted) {
                m_robotDrive.drive(0, 0, 0, false);
        }

        public boolean isFinished() {
                if (Math.abs(rError) < 0.05) {
                        return true;
                }
                return false;
                // stops the function by returning true
        }
}