package frc.robot.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;

public class Limelight extends SubsystemBase {

    /** Creates a new Subsystem. */
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ta = table.getEntry("ta");
    NetworkTableEntry tv = table.getEntry("tv");

    double x;
    double y;
    double v;
    double area;

    boolean hasValidTarget = false;

    // calculate angles
    double angleToGoalDegrees = LimelightConstants.limelightMounAngleDegrees + y;
    double angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180.0);

    // calculate distance
    double distanceFromLimelightToGoalInches = ((LimelightConstants.goalHeightMeters - LimelightConstants.limelightLensHeightMeters)/Math.tan(angleToGoalRadians));

    public Limelight() {
        v = table.getEntry("tv").getDouble(0);
        setValidTarget(v);
    }

    // proportional control constant for distance
    public void periodic() {
        v = tv.getDouble(0);
        // Check for Valid target
        setValidTarget(v);

        x = tx.getDouble(0.0);
        y = ty.getDouble(0.0);
        area = ta.getDouble(0.0);

        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("LimelightArea", area);
        SmartDashboard.putBoolean("Has Valid Target", hasValidTarget);

        distanceFromLimelightToGoalInches = 
        ((LimelightConstants.goalHeightMeters - LimelightConstants.limelightLensHeightMeters) / Math.tan((angleToGoalDegrees+y) * Math.PI / 180));
    }

    public double getDis() {
        return distanceFromLimelightToGoalInches;
    }

    public double getX(){
        return x;
    }
    public double getY(){
        return y;
    }
    public double getV(){
        return v;
    }

    public void setValidTarget(double v) {
        if (v == 0) {
            hasValidTarget = false;
        } else if (v == 1) {
            hasValidTarget = true;
        }
    }

    public boolean getTargetStatus() {
        return hasValidTarget;
    }
}