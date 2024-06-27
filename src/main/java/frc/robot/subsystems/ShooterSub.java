// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSub extends SubsystemBase {
  CANSparkMax LeftUpperShooter = new CANSparkMax(Constants.ShooterConstants.LeftUpperShooterID, MotorType.kBrushless);
  CANSparkMax LeftLowerShooter = new CANSparkMax(Constants.ShooterConstants.LeftLowerShooterID, MotorType.kBrushless);
  CANSparkMax RightUpperShooter = new CANSparkMax(Constants.ShooterConstants.RightUpperShooterID, MotorType.kBrushless);
  CANSparkMax RightLowerShooter = new CANSparkMax(Constants.ShooterConstants.RightLowerShooterID, MotorType.kBrushless);
  RelativeEncoder encoder;
  DigitalInput IntakeLimiterSwitch = new DigitalInput(Constants.ShooterConstants.IntakeLimiterSwitch);

  /** Creates a new LaunchSub. */
  public ShooterSub() {
    encoder = LeftUpperShooter.getEncoder();
  }

  public void setLaunchMotors(double lowerShooterSpeed, double upperShooterSpeed) {
    if (IntakeLimiterSwitch.get() == true) {
      LeftLowerShooter.set(lowerShooterSpeed);
      LeftUpperShooter.set(upperShooterSpeed);
    } else {
      LeftLowerShooter.set(MathUtil.clamp(lowerShooterSpeed, 0.0, 1.0));
      LeftLowerShooter.set(MathUtil.clamp(upperShooterSpeed, 0.0, 1.0));
    }
    RightLowerShooter.follow(LeftLowerShooter);
    RightUpperShooter.follow(LeftUpperShooter);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("top motor velocity", Velocity());
    // This method will be called once per scheduler run
  }
  public double Velocity() {
    return encoder.getVelocity();
  }
}
