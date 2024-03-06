// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubSystem;
import frc.robot.subsystems.UltraSonicSub;

public class UltrasonicCmd extends Command {
  UltraSonicSub m_UltraSonicSub;
  boolean isRobotOrientationEven;
  double currentLSonicIn;
  double currentRSonicIn;
  SwerveSubSystem m_SwerveSub;
  int distanceFromObject;

  public UltrasonicCmd(UltraSonicSub ultraSonicSub, SwerveSubSystem swerveSub, int inchesFromObject) {
    m_UltraSonicSub = ultraSonicSub;
    m_SwerveSub = swerveSub;
    distanceFromObject = inchesFromObject;

    addRequirements(m_UltraSonicSub);
    addRequirements(m_SwerveSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_SwerveSub.drive(0, 0, 0);
    isRobotOrientationEven = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentLSonicIn = m_UltraSonicSub.LSonicPoll;
    currentRSonicIn = m_UltraSonicSub.RSonicPoll;
    if (MathUtil.applyDeadband(currentLSonicIn, 0.5) != distanceFromObject) {
      m_SwerveSub.drive(0, Constants.sonicConstants.AutoMapSpeed, 0);
      isRobotOrientationEven = false;
    } else if (MathUtil.applyDeadband(currentRSonicIn, 0.5) != distanceFromObject) {
      isRobotOrientationEven = false;
      m_SwerveSub.drive(0, Constants.sonicConstants.AutoMapSpeed, 0);
    } else {
      m_SwerveSub.drive(0, 0, 0);
      isRobotOrientationEven = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_SwerveSub.drive(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isRobotOrientationEven;
  }
}
