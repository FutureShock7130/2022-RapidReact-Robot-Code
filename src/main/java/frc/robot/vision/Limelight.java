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

    // calculate angles
    double angleToGoalDegrees = LimelightConstants.limelightMounAngleDegrees + y;
    double angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180.0);

    // calculate distance
    double distanceFromLimeligtToGoalInches = ((LimelightConstants.goalHeightMeters - LimelightConstants.limelightLensHeightMeters)/Math.tan(angleToGoalRadians));

    // proportional control constant for distance
    public void periodic() {
        x = tx.getDouble(0.0);
        y = ty.getDouble(0.0);
        area = ta.getDouble(0.0);
        v = tv.getDouble(0.0);

        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("LimelightArea", area);

        distanceFromLimeligtToGoalInches = 
        ((LimelightConstants.goalHeightMeters - LimelightConstants.limelightLensHeightMeters) / Math.tan((angleToGoalDegrees+y) * Math.PI / 180));
    }

    public double getDis() {
        return distanceFromLimeligtToGoalInches;
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
}