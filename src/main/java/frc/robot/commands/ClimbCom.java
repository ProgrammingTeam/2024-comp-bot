// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants;
import frc.robot.subsystems.ClimbSub;

public class ClimbCom extends Command {
  private final ClimbSub m_ClimbSub;
  private final double m_ClimbSpeed;
  private double TargetEncoderValue;
  private final CommandJoystick leftJoystick;

  /** Creates a new ClimbCom. */
  public ClimbCom(ClimbSub Climb, double climbSpeed, CommandJoystick Left) {
    m_ClimbSpeed = climbSpeed;
    m_ClimbSub = Climb;
    leftJoystick = Left;
    addRequirements(m_ClimbSub);
    TargetEncoderValue = MathUtil.interpolate(Constants.Climb.BottomEncoderPosition, Constants.Climb.TopEncoderPosition,
        (leftJoystick.getRawAxis(3) + 1) / -2);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_ClimbSub.setMotors(0, 0);
    SmartDashboard.putNumber("Target climb", TargetEncoderValue);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    TargetEncoderValue = MathUtil.interpolate(Constants.Climb.BottomEncoderPosition, Constants.Climb.TopEncoderPosition,
        (leftJoystick.getRawAxis(3) + 1) / -2);
    if (Math.abs(m_ClimbSub.EncoderValue() - TargetEncoderValue) <= Constants.Climb.ClimberEncoderDeadban) {
      m_ClimbSub.setMotors(0, 0);
    } else if (m_ClimbSub.EncoderValue() >= TargetEncoderValue) {
      m_ClimbSub.setMotors(m_ClimbSpeed, m_ClimbSpeed);
    } else if (m_ClimbSub.EncoderValue() <= TargetEncoderValue) {
      m_ClimbSub.setMotors(-m_ClimbSpeed, -m_ClimbSpeed);
    } else {
      m_ClimbSub.setMotors(0, 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ClimbSub.setMotors(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if (Math.abs(m_ClimbSub.EncoderValue() - TargetEncoderValue) <=
    // Constants.Climb.ClimberEncoderDeadban) {
    // return true;
    // }

    // else {
    // return false;
    // }
    // }
    return false;
  }
}
