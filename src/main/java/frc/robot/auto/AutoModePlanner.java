package frc.robot.auto;

public class AutoModePlanner {
    private enum DriveMode {
        MECANUM,
        DIFFERENTIAL,
        ARCADE,
        TANK
    }

    private enum AimMode {
        LIMELIGHT, 
        ABSOLUTE,
        RELATIVE
    }

    private enum Alliance {
        RED_ALLIANCE,
        BLUE_ALLIANCE
    }

    private enum DrivePath {
        ONE_CARGO,
        TWO_CARGO,
        THREE_CARGO,
        FOUR_CARGO,
        FIVE_CARGO
    }
}