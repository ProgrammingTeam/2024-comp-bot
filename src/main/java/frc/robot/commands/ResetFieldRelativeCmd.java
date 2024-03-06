// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubSystem;

public class ResetFieldRelativeCmd extends Command {
  SwerveSubSystem m_swerveSub;
  double currentRobotGyro;
  boolean isReset;

  public ResetFieldRelativeCmd(SwerveSubSystem swerveSub) {
    m_swerveSub = swerveSub;

    addRequirements(m_swerveSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentRobotGyro = m_swerveSub.getRobotOrientation();
    System.out.println("Current gyro is at " + currentRobotGyro);
    isReset = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (currentRobotGyro != 0) {
      System.out.println("Reseting gyro");
      m_swerveSub.resetGyro();
      isReset = true;
    } else {
      System.out.println("Gyro was already at 0");
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Reseting gyro command was interrupted");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isReset;
  }
}
