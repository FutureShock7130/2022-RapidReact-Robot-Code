package frc.robot.statemachines;

public class DriveFSM {

    public enum DriveOdometryState {
        MECANUM_ODOMETRY,
        DIFFERENTIAL_ODOMETRY
    }
    
    private DriveOdometryState odometryState = DriveOdometryState.MECANUM_ODOMETRY;

    public DriveOdometryState getCurrentOdometry() {
        return odometryState;
    }

    public void setOdometryMecanum() {
        odometryState = DriveOdometryState.MECANUM_ODOMETRY;
    }

    public void setOdometryDifferential() {
        odometryState = DriveOdometryState.DIFFERENTIAL_ODOMETRY;
    }
}
