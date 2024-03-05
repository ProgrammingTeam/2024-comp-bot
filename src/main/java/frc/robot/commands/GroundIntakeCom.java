// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.GroundIntakeSub;

public class GroundIntakeCom extends Command {
  private final GroundIntakeSub m_GroundIntakeSub;
  private final double m_MotorSpeed;

  /** Creates a new GroundIntakeCom. */
  public GroundIntakeCom(GroundIntakeSub GIntakeSub, double Speed) {
    m_GroundIntakeSub = GIntakeSub;
    m_MotorSpeed = Speed;
    addRequirements(m_GroundIntakeSub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_GroundIntakeSub.setMotors(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_GroundIntakeSub.setMotors(-m_MotorSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_GroundIntakeSub.setMotors(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
