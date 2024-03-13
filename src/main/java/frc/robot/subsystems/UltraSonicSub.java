// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class UltraSonicSub extends SubsystemBase {
  // public final Ultrasonic LSonic = new Ultrasonic(Constants.sonicConstants.LSonicPingPort,
  //     Constants.sonicConstants.LSonicEchoPort);
  // public final Ultrasonic RSonic = new Ultrasonic(Constants.sonicConstants.RSonicPingPort,
  //     Constants.sonicConstants.RSonicEchoPort);

  private final AnalogInput LSonic = new AnalogInput(Constants.sonicConstants.LSonicPort);
  private final AnalogInput RSonic = new AnalogInput(Constants.sonicConstants.RSonicPort); 


  public UltraSonicSub() { 
  }

  public double getLeftRangeIn() {
    return LSonic.getVoltage() * Constants.sonicConstants.voltageToInchesConversionFactor;
  }

  public double getRightRangeIn() {
    return RSonic.getVoltage() * Constants.sonicConstants.voltageToInchesConversionFactor;
  }

  @Override
  public void periodic() {
  }
}
