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
  boolean isLeft;
  Timer timer;
  Timer timer2;
  double currentRot;

  public AutoDrive(SwerveSubSystem m_swervesub, int seconds, boolean forward, double rotationDegree,
      boolean leftOrRight) {
    swerveSub = m_swervesub;
    time = seconds;
    isforward = forward;
    isLeft = leftOrRight;
    rot = rotationDegree;

    addRequirements(swerveSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer2.reset();
    swerveSub.drive(0, 0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // First rotate until the degree is met, then start a timer and move until timer
    // reaches set limit.
    currentRot = swerveSub.encoderPoll;
    if (currentRot <= rot) {
      swerveSub.drive(0, 0, Constants.autonomousConstants.RotationSpeed);
    } else {
      timer.start();
      if (isforward == true && timer.get() <= time) {
        swerveSub.drive(0, Constants.autonomousConstants.FrontOrBackSpeed, 0);
        isMet = false;
      } else if (isforward == false && timer.get() <= time) {
        swerveSub.drive(0, -Constants.autonomousConstants.FrontOrBackSpeed, 0);
        isMet = false;
      } else {
        swerveSub.drive(0, 0, 0);
        isMet = false;
      }
      timer2.start();
      if (isLeft == true) {
        if (timer2.get() <= time) {
          swerveSub.drive(Constants.autonomousConstants.LeftToRightSpeed, 0, 0);
        } else {
          swerveSub.drive(0, 0, 0);
          isMet = true;
        }
      } else if (isLeft == false) {
        if (timer2.get() <= time) {
          swerveSub.drive(-Constants.autonomousConstants.LeftToRightSpeed, 0, 0);
        } else {
          swerveSub.drive(0, 0, 0);
          isMet = true;
        }
      }
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
