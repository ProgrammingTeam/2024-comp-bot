// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbSub extends SubsystemBase {
  private final CANSparkMax LeftClimbMotor = new CANSparkMax(Constants.LeftClimbMotorID, MotorType.kBrushless);
  private final CANSparkMax RightClimbMotor = new CANSparkMax(Constants.RightClimbMotorID, MotorType.kBrushless);
  
  
  /** Creates a new ClimbSub. */
  public ClimbSub() {
    LeftClimbMotor.getAbsoluteEncoder();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void setMotors(double ClimbSpeed) {
    LeftClimbMotor.set(ClimbSpeed);
    RightClimbMotor.follow(LeftClimbMotor);
  }
  public double EncoderValue() {
    return LeftClimbMotor.get();
  }
}
