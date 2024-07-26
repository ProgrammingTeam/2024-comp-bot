// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
//import edu.wpi.first.math.MathUtil;
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

    LeftLowerShooter.restoreFactoryDefaults();
    LeftUpperShooter.restoreFactoryDefaults();
    RightLowerShooter.restoreFactoryDefaults();
    RightUpperShooter.restoreFactoryDefaults();

    LeftLowerShooter.setIdleMode(IdleMode.kCoast);
    LeftUpperShooter.setIdleMode(IdleMode.kCoast);
    RightLowerShooter.setIdleMode(IdleMode.kCoast);
    RightUpperShooter.setIdleMode(IdleMode.kCoast);

    LeftLowerShooter.setSmartCurrentLimit(80);
    LeftUpperShooter.setSmartCurrentLimit(80);
    RightLowerShooter.setSmartCurrentLimit(80);
    RightUpperShooter.setSmartCurrentLimit(80);   
    
    RightLowerShooter.follow(LeftLowerShooter,true);
    RightUpperShooter.follow(LeftUpperShooter,true);

    LeftLowerShooter.burnFlash();
    LeftUpperShooter.burnFlash();
    RightLowerShooter.burnFlash();
    RightUpperShooter.burnFlash(); 
  }

  public void setLaunchMotors(double lowerShooterSpeed, double upperShooterSpeed) {
       
LeftLowerShooter.set(lowerShooterSpeed);
      LeftUpperShooter.set(upperShooterSpeed);

    // if (IntakeLimiterSwitch.get() == true) {
    //   LeftLowerShooter.set(lowerShooterSpeed);
    //   LeftUpperShooter.set(upperShooterSpeed);
    //   // RightLowerShooter.set(-lowerShooterSpeed);
    //   // RightUpperShooter.set(-upperShooterSpeed);
    // } else {
    //   LeftLowerShooter.set(MathUtil.clamp(lowerShooterSpeed, 0.0, 1.0));
    //   LeftLowerShooter.set(MathUtil.clamp(upperShooterSpeed, 0.0, 1.0));
    // } 
  
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
