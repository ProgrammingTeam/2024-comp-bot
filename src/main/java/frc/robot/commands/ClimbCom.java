// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ClimbSub;

public class ClimbCom extends Command {
  private final ClimbSub m_ClimbSub;
  private final double m_ClimbSpeed;
  private final double TargetEncoderValue;

  /** Creates a new ClimbCom. */
  public ClimbCom(ClimbSub Climb, double climbSpeed, double Height) {
    m_ClimbSpeed = climbSpeed;
    m_ClimbSub = Climb;
    addRequirements(m_ClimbSub);

    TargetEncoderValue = MathUtil.interpolate(Constants.Climb.BottomEncoderPosition, Constants.Climb.TopEncoderPosition,
        Height);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_ClimbSub.setMotors(m_ClimbSpeed);

    if (m_ClimbSub.EncoderValue() >= TargetEncoderValue) {

    } else if (m_ClimbSub.EncoderValue() <= TargetEncoderValue) {
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(m_ClimbSub.EncoderValue() - TargetEncoderValue) <= Constants.Climb.ClimberEncoderDeadban) {
      return true;
    }

    else {
      return false;
    }
  }
}
