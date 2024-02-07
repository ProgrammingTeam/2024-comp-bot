// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonomousCmds;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.AutonomousConstants;
import frc.robot.subsystems.SwerveSubSystem;

public class AutoRot extends Command {
  Timer timer;
  SwerveSubSystem SwerveSub;
  double Y;
  double X;
  double rot;
  boolean TimeMet;
  boolean Left;
  int TotalTime;

  public AutoRot(SwerveSubSystem swerveSubSystem, int TimeInput, boolean isLeft) {
    SwerveSub = swerveSubSystem;
    TotalTime = TimeInput;
    Left = isLeft;

    addRequirements(SwerveSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    X = AutonomousConstants.autoX;
    rot = AutonomousConstants.autoRotation;
    Y = AutonomousConstants.autoY;
    TimeMet = false;
    timer = new Timer();
    timer.start();
    SwerveSub.drive(0, 0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Left == true) {
      if (timer.get() <= TotalTime) {
        SwerveSub.drive(Y, X, rot);
        Commands.waitSeconds(TotalTime);
        TimeMet = true;
      } else {
        SwerveSub.drive(0, 0, 0);
      }
    } else if (Left == false) {
      if (timer.get() <= TotalTime) {
        SwerveSub.drive(-Y, -X, -rot);
        Commands.waitSeconds(TotalTime);
        TimeMet = true;
      } else {
        SwerveSub.drive(0, 0, 0);
      }
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
    return TimeMet;
  }
}
