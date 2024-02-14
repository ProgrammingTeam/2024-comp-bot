// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.cameraserver.CameraServer;
public class CameraSub extends SubsystemBase { 
 
  /** Creates a new CameraSub. */
  public CameraSub() {
    CameraServer.startAutomaticCapture("Drive CAM", 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
