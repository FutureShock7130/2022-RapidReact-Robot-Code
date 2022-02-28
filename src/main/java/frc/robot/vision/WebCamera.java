// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class WebCamera extends SubsystemBase {

  /** Creates a new WebCamera. */
  public WebCamera() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Creates UsbCamera and MjpegServer [1] and connects them
    UsbCamera camera = CameraServer.startAutomaticCapture();
    // Creates the CvSink and connects it to the UsbCamera
    CvSink cvSink = CameraServer.getVideo();
    // Creates the CvSource and MjpegServer [2] and connects them
    CvSource outputStream = CameraServer.putVideo("Blur", 640, 480);
  }
}
