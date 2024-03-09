// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.MotorConstants;
import frc.robot.subsystems.ShooterSub;

public class ShootCmd extends Command {
  ShooterSub Shooter;
  ShootModes ShootSelection;
  double BottomMotor;
  double TopMotor;
  double FireVelocity;

  public ShootCmd(ShooterSub sub, ShootModes mode, double rpmToFire) {
    Shooter = sub;
    ShootSelection = mode;
    FireVelocity = rpmToFire;
    addRequirements(Shooter);
  }

  public ShootCmd(ShooterSub ShooterSystem, ShootModes Mode) {
    ShootSelection = Mode;
    Shooter = ShooterSystem;
    FireVelocity = Constants.ShooterConstants.DefaultShootVelocity;
    addRequirements(Shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Shooter.setLaunchMotors(0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (ShootSelection) {
      case Shoot:
        BottomMotor = Constants.ShooterConstants.InteriorShooterSpeed;
        TopMotor = Constants.ShooterConstants.ExteriorShooterSpeed;
        break;

      case Load:
        BottomMotor = -Constants.ShooterConstants.IntakeShooterSpeed;
        TopMotor = -Constants.ShooterConstants.IntakeShooterSpeed;
        BottomMotor = MotorConstants.InteriorShooterSpeed;
        TopMotor = MotorConstants.ExteriorShooterSpeed;
        SmartDashboard.putString("Current Shooter Function", "Shooting");
        break;

      case SpinUp:
        BottomMotor = 0;
        TopMotor = Constants.ShooterConstants.ExteriorShooterSpeed;
        SmartDashboard.putString("Current Shooter Function", "Spinning Up");
        break;

      case AmpShot:
        BottomMotor = 0.16;
        TopMotor = 0.16;
        SmartDashboard.putString("Current Shooter Function", "Amp Shot");
        break;

      case SmartShoot:
        if (Shooter.Velocity() <= FireVelocity) {
          BottomMotor = 0;
          TopMotor = Constants.ShooterConstants.ExteriorShooterSpeed;
        } else {
          BottomMotor = Constants.ShooterConstants.InteriorShooterSpeed;
          TopMotor = Constants.ShooterConstants.ExteriorShooterSpeed;
        }
        break;

      case NOTHING:
        BottomMotor = 0;
        TopMotor = 0;
        SmartDashboard.putString("Current Shooter Function", "Idle");
        break;

      default:
        BottomMotor = 0;
        TopMotor = 0;
        break;
    }
    Shooter.setLaunchMotors(BottomMotor, TopMotor);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Shooter.setLaunchMotors(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public enum ShootModes {
    SpinUp,
    Shoot,
    Load,
    AmpShot,
    SmartShoot,
    NOTHING;
  }

}
