// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class tankDrive extends SubsystemBase {
  /** Creates a new tankDrive. */
   final CANSparkMax leftMotor = new CANSparkMax(Constants.leftMotorID, MotorType.kBrushless);
   final CANSparkMax rightMotor = new CANSparkMax(Constants.rightMotorID, MotorType.kBrushless);
  
   public tankDrive() {
    rightMotor.setInverted(true);
   }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setMotors(double leftSpeed, double rightSpeed) {
    leftMotor.set(leftSpeed);
    rightMotor.set(rightSpeed);
  }
}
