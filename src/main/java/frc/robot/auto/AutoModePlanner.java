package frc.robot.auto;

public class AutoModePlanner implements AutoModes {
    private AutoModes.DriveStrategy driveStrategy = DriveStrategy.ONE_CARGO;
    private AutoModes.Alliance alliance = Alliance.BLUE_ALLIANCE;
    private AutoModes.DriveMode driveMode = DriveMode.DIFFERENTIAL;
    private AutoModes.StartingPosition startingPos = StartingPosition.ONE;
    private AutoModes.AimMode aimMode = AimMode.LIMELIGHT;

    public void handleAutoMode() {
        // Write Conditional Logic to decide which Autononous Path etc... to go on.
    }
}