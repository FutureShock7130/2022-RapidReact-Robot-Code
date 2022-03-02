package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Telemetry {

    NetworkTableInstance instTable;

    NetworkTable visionTable;
    NetworkTableEntry visionXEntry;

    private boolean isAuto = false;
    private boolean isTeleop = false;

    public Telemetry() {
        // Getting all the Tables that are needed
        instTable = NetworkTableInstance.getDefault();
        visionTable = instTable.getTable("limelight");

        // Vision Entries
        visionXEntry = visionTable.getEntry("vx");
    }

    public synchronized void handle() {
        visionXEntry.getDouble(0.0);
    }

    public void setMode(String mode) {
        if (mode == "Auto") {
            isAuto = true;
            isTeleop = false;
            return;
        } else if (mode == "Tele") {
            isAuto = false;
            isTeleop = true;
            return;
        }
    }
}
