// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class UltraSonicSub extends SubsystemBase {
  public final Ultrasonic LSonic = new Ultrasonic(Constants.sonicConstants.LSonicPingPort,
      Constants.sonicConstants.LSonicEchoPort);
  public final Ultrasonic RSonic = new Ultrasonic(Constants.sonicConstants.RSonicPingPort,
      Constants.sonicConstants.RSonicEchoPort);


  public UltraSonicSub() {
  }

  public double getLeftRangeIn() {
    return LSonic.getRangeInches();
  }

  public double getRightRangeIn() {
    return RSonic.getRangeInches();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Left Sonic Distence", getLeftRangeIn());
    SmartDashboard.putNumber("Right Sonic Distence", getRightRangeIn());
  }
}
