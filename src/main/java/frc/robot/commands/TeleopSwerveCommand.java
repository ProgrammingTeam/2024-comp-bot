// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.subsystems.SwerveSubSystem;

public class TeleopSwerveCommand extends Command {
  SwerveSubSystem m_swerveSubSystem;
  CommandJoystick m_LJoystick;
  CommandJoystick m_RJoystick;

  public TeleopSwerveCommand(SwerveSubSystem swerveSubSystem, CommandJoystick LeftController,
      CommandJoystick RightController) {
    m_swerveSubSystem = swerveSubSystem;
    m_LJoystick = LeftController;
    m_RJoystick = RightController;
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
      Math.pow(-m_LJoystick.getRawAxis(0), 3) * ((-m_RJoystick.getRawAxis(3) + 1) / 2),
      Math.pow(-m_LJoystick.getRawAxis(1), 3) * ((-m_RJoystick.getRawAxis(3) + 1) / 2),
      Math.pow(-m_RJoystick.getRawAxis(0), 3) * ((-m_RJoystick.getRawAxis(3) + 1) / 2));
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
