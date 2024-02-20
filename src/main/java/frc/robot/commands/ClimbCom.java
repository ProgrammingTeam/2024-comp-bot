// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSub;

public class ClimbCom extends Command {
  private final ClimbSub m_ClimbSub;
  private final double m_ClimbSpeed;
  private final double m_height;
  /** Creates a new ClimbCom. */
  public ClimbCom(ClimbSub Climb, double climbSpeed, double Height) {
    m_ClimbSpeed = climbSpeed;
    m_ClimbSub = Climb;
    m_height = Height;
    addRequirements(m_ClimbSub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_ClimbSub.setMotors(m_ClimbSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_ClimbSub.EncoderValue() >= m_height -0.5 && m_ClimbSub.EncoderValue() <= m_height + 0.5;
  }
}
