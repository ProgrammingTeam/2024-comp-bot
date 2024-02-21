// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubSystem;

public class AutoDrive extends Command {
  SwerveSubSystem swerveSub;
  int time;
  boolean isforward;
  double rot;
  boolean isMet;
  Timer timer;

  public AutoDrive(SwerveSubSystem m_swervesub, int seconds, boolean forward, double rotationSpeed) {
    swerveSub = m_swervesub;
    time = seconds;
    isforward = forward;
    rot = rotationSpeed;

    addRequirements(swerveSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
    swerveSub.drive(0, 0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (isforward == true | timer.get() <= time) {
      swerveSub.drive(0, Constants.autonomousConstants.FrontOrBackSpeed, rot);
      isMet = false;
    } else if (isforward == false | timer.get() <= time) {
      swerveSub.drive(0, -Constants.autonomousConstants.FrontOrBackSpeed, rot);
      isMet = false;
    } else {
      swerveSub.drive(0, 0, 0);
      isMet = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSub.drive(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isMet;
  }
}
