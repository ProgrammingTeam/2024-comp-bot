// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.SwerveSubSystem;

public class xboxTeleopSwerveCom extends Command {
  SwerveSubSystem m_swerveSubSystem;

  CommandXboxController m_XboxController;

  public xboxTeleopSwerveCom(SwerveSubSystem swerveSubSystem, CommandXboxController m_driverController) {
    m_swerveSubSystem = swerveSubSystem;
    m_XboxController = m_driverController;
    addRequirements(m_swerveSubSystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_swerveSubSystem.drive(
      Math.pow(-m_XboxController.getRawAxis(0), 3) * (1 - m_XboxController.getRawAxis(3)),
      Math.pow(-m_XboxController.getRawAxis(1), 3) * (1 - m_XboxController.getRawAxis(3)),
      Math.pow(-m_XboxController.getRawAxis(4), 3) * (1 - m_XboxController.getRawAxis(3)));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
