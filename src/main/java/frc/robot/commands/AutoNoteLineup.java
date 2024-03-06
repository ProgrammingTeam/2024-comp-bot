// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubSystem;

public class AutoNoteLineup extends Command {
  private final SwerveSubSystem m_SwerveSubSystem;
  private boolean LinedUp;
  private double finalOrientation;

  /** Creates a new AutoNoteLineup. */
  public AutoNoteLineup(SwerveSubSystem SwerveSub) {
    m_SwerveSubSystem = SwerveSub;
    addRequirements(m_SwerveSubSystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    LinedUp = false;
    finalOrientation = m_SwerveSubSystem.getRobotOrientation() - Constants.OrientaionOffset;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!MathUtil.isNear(finalOrientation, m_SwerveSubSystem.getRobotOrientation(), 1)) {
      m_SwerveSubSystem.drive(0, 0, -Constants.AutoTurnSpeed);
    } else {
      LinedUp = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return LinedUp;
  }
}
