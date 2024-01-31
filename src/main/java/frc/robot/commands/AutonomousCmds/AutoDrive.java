// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonomousCmds;

import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.SwerveSubSystem;

public class AutoDrive extends Command {
  SwerveSubSystem SwerveSub;
  double leftYValue;
  double leftXValue;
  double rightXValue;
  boolean DistanceMet;
  boolean Forward;

  public AutoDrive(SwerveSubSystem swerveSubSystem, boolean isForward) {
    SwerveSub = swerveSubSystem;
    Forward = isForward;

    addRequirements(SwerveSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SwerveSub.drive(0, 0, 0);
    DistanceMet = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Forward == true) {
      SwerveSub.drive(leftYValue, leftXValue, rightXValue);
      Commands.waitSeconds(3);
      DistanceMet = true;
    } else {
      SwerveSub.drive(-leftYValue, -leftXValue, -rightXValue);
      Commands.waitSeconds(3);
      DistanceMet = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SwerveSub.drive(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return DistanceMet;
  }
}
