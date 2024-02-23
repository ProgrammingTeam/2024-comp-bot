// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.ClimbCom;

public class ClimbSub extends SubsystemBase {
  private final CANSparkMax LeftClimbMotor = new CANSparkMax(Constants.Climb.LeftClimbMotorID, MotorType.kBrushless);
  private final CANSparkMax RightClimbMotor = new CANSparkMax(Constants.Climb.RightClimbMotorID, MotorType.kBrushless);
  private final SparkAbsoluteEncoder PhysicalEncoderValue;

  /** Creates a new ClimbSub. */
  public ClimbSub() {
    PhysicalEncoderValue = LeftClimbMotor.getAbsoluteEncoder();
    RightClimbMotor.follow(LeftClimbMotor);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Encoder Value", EncoderValue());
  }

  public void setMotors(double ClimbSpeed) {
    LeftClimbMotor.set(ClimbSpeed);
  }

  public double EncoderValue() {
    return PhysicalEncoderValue.getPosition();
  }
}
