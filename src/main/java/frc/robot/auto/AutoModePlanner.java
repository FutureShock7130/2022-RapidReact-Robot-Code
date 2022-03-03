package frc.robot.auto;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.auto.Paths.FourCargoFromOne;
import frc.robot.auto.Paths.FourCargoFromThree;
import frc.robot.auto.Paths.ThreeCargoFromOne;
import frc.robot.auto.Paths.ThreeCargoFromThree;
import frc.robot.auto.Paths.ThreeCargoFromTwo;
import frc.robot.auto.Paths.TwoCargoFromOne;
import frc.robot.auto.Paths.TwoCargoFromThree;
import frc.robot.auto.Paths.TwoCargoFromTwo;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Spinner;
import frc.robot.subsystems.Transporter;
import frc.robot.vision.Limelight;

public class AutoModePlanner implements AutoModes {
    private AutoModes.DriveStrategy driveStrategy = DriveStrategy.ONE_CARGO;
    private AutoModes.Alliance alliance = Alliance.BLUE_ALLIANCE;
    private AutoModes.DriveMode driveMode = DriveMode.DIFFERENTIAL;
    private AutoModes.StartingPosition startingPos = StartingPosition.ONE;
    private AutoModes.AimMode aimMode = AimMode.LIMELIGHT;

    private RobotContainer robot;

    public AutoModePlanner(
        RobotContainer robotContainer
    ) {
        robot = robotContainer;
    }

    public SequentialCommandGroup handleAutoMode() {
        switch (startingPos) {
            case ONE:
                switch (driveStrategy) {
                    case TWO_CARGO:
                        return new TwoCargoFromOne(robot).getCommand();
                    case THREE_CARGO:
                        return new ThreeCargoFromOne(robot).getCommand();
                    case FOUR_CARGO:
                        return new FourCargoFromOne(robot).getCommand();
                    case FIVE_CARGO:
                        break;
                }
            case TWO:
                switch (driveStrategy) {
                    case TWO_CARGO:
                        return new TwoCargoFromTwo(robot).getCommand();
                    case THREE_CARGO:
                        return new ThreeCargoFromTwo(robot).getCommand();
                    case FOUR_CARGO:
                        // return new FourCargoFromTwo(robot).getCommand();
                    case FIVE_CARGO:
                        break;
                }
            case THREE:
                switch (driveStrategy) {
                    case TWO_CARGO:
                        return new TwoCargoFromThree(robot).getCommand();
                    case THREE_CARGO:
                        return new ThreeCargoFromThree(robot).getCommand();
                    case FOUR_CARGO:
                        // return new FourCargoFromThree(robot).getCommand();
                    case FIVE_CARGO:
                        break;
                }

            default:
                // return a generic shoot command here
                return new TwoCargoFromOne(robot).getCommand();
        }
    }

    public void setCargoCount(AutoModes.DriveStrategy cargo) {
        driveStrategy = cargo;
    }

    public void setAlliance(AutoModes.Alliance _alliance) {
        alliance = _alliance;
    }

    public void setAimMode(AutoModes.AimMode mode) {
        aimMode = mode;
    }
}