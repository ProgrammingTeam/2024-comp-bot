// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.LimelightSub;
import frc.robot.subsystems.tankDrive;

public class LineUpCom extends Command {
  private final tankDrive m_TankSub;
  private final LimelightSub m_LimelightSub;
  private boolean Centered;
  /** Creates a new LineUpCom. */
  public LineUpCom(tankDrive TankSub, LimelightSub LimeSub) {
    m_TankSub = TankSub;
    m_LimelightSub = LimeSub;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_TankSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Centered = false;
    m_TankSub.setMotors(0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_LimelightSub.getTarget() == -1) {
     m_TankSub.setMotors(0, 0);
      return;
    }
    if(m_LimelightSub.angleFromCenter() < -1) {
      m_TankSub.setMotors(-0.04, 0.05);
    }
    else if(MathUtil.applyDeadband(m_LimelightSub.angleFromCenter(), 1) == 0.0) {
      Centered = true;
      m_TankSub.setMotors(0, 0);
    }
    else {
      m_TankSub.setMotors(0.04, -0.04);
    } 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_TankSub.setMotors(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Centered;
  }
}
