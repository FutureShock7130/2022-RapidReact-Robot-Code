package frc.robot.auto;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.RobotContainer;
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

    public Command handleAutoMode() {
        switch (startingPos) {
            case ONE:
                switch (driveStrategy) {
                    case TWO_CARGO:
                        // return the two cargo from pos 1
                    case THREE_CARGO:
                        // for the rest of the logic, follow the one above
                    case FOUR_CARGO:
                    case FIVE_CARGO:
                }
            case TWO:
                switch (driveStrategy) {
                    case TWO_CARGO:
                    case THREE_CARGO:
                    case FOUR_CARGO:
                    case FIVE_CARGO:
                }
            case THREE:
                switch (driveStrategy) {
                    case TWO_CARGO:
                    case THREE_CARGO:
                    case FOUR_CARGO:
                    case FIVE_CARGO:
                }

            default:
                // return a generic shoot command here
                return new RunCommand(toRun, requirements);
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