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
  private final UltraSonicSub m_UltraSonicSub;
  private boolean isRobotOrientationEven;
  private final SwerveSubSystem m_SwerveSub;

  public UltrasonicCmd(UltraSonicSub ultraSonicSub, SwerveSubSystem swerveSub) {
    m_UltraSonicSub = ultraSonicSub;
    m_SwerveSub = swerveSub;

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
    if(MathUtil.isNear(m_UltraSonicSub.getLeftRangeIn(), m_UltraSonicSub.getRightRangeIn(), 0.1)) {
      m_SwerveSub.drive(0, 0, .2);
    }
    else {
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
