// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.LimelightSub;
import frc.robot.subsystems.SwerveSubSystem;

public class SpeakerLimLineupCom extends Command {
  private final LimelightSub m_LimelightSub;
  private final SwerveSubSystem m_SwerveSubSystem;
  private boolean linedUp = false;
  /** Creates a new LimLineupCom. */
  public SpeakerLimLineupCom(LimelightSub LimSub, SwerveSubSystem SwerveSub) {
    m_LimelightSub = LimSub;
    m_SwerveSubSystem = SwerveSub;
    addRequirements(m_SwerveSubSystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    linedUp = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_LimelightSub.getTarget() != 4 || m_LimelightSub.getTarget() != 7) {
      m_SwerveSubSystem.drive(0, 0, Constants.AutoConstants.AutoTurnSpeed);
    }
    else {
      if (MathUtil.applyDeadband(m_LimelightSub.angleFromCenter(), 1) > 0) {
        m_SwerveSubSystem.drive(0, 0, Constants.AutoConstants.AutoTurnSpeed);
      }
      else if (MathUtil.applyDeadband(m_LimelightSub.angleFromCenter(), 1) < 0) {
        m_SwerveSubSystem.drive(0, 0, -Constants.AutoConstants.AutoTurnSpeed);
      }
      else {
        linedUp = true;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return linedUp;
  }
}
