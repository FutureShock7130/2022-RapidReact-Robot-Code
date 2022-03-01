package frc.robot.auto;

public interface AutoModes {
    public enum DriveMode {
        MECANUM,
        DIFFERENTIAL,
    }

    public enum AimMode {
        LIMELIGHT, 
        ABSOLUTE,
        RELATIVE
    }

    public enum Alliance {
        RED_ALLIANCE,
        BLUE_ALLIANCE
    }

    public enum StartingPosition {
        ONE,
        TWO,
        THREE
    }

    public enum DriveStrategy {
        ONE_CARGO,
        TWO_CARGO,
        THREE_CARGO,
        FOUR_CARGO,
        FIVE_CARGO
    }
}
