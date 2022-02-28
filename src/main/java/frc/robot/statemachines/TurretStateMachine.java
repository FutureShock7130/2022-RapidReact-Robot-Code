package frc.robot.statemachines;

public class TurretStateMachine {
    public enum TurretMode {
        LIMELIGHT_AIM,
        RELATIVE_AIM,
        ABSOLUTE_AIM
    }

    private TurretMode turretMode = TurretMode.LIMELIGHT_AIM;

    public TurretMode getTurretMode() {
        return turretMode;
    }
}