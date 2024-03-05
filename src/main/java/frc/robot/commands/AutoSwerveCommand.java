// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubSystem;

public class AutoSwerveCommand extends Command {
  private final SwerveSubSystem m_swerveSubSystem;
  private final double m_YMovement;
  private final double m_XMovement;
  private final double m_distanceNeeded;
  private double distanceTraveled;

  public AutoSwerveCommand(SwerveSubSystem swerveSubSystem, double YMove, double XMove, double distance) {
    m_swerveSubSystem = swerveSubSystem;
    m_YMovement = YMove;
    m_XMovement = XMove;
    m_distanceNeeded = distance;
    addRequirements(m_swerveSubSystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    distanceTraveled = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Rotation value subject to change
    m_swerveSubSystem.drive(m_YMovement, m_XMovement, 0);
    distanceTraveled += Units.metersToInches(m_swerveSubSystem.metersPSec/50);
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return MathUtil.isNear(m_distanceNeeded, distanceTraveled, Constants.InchesTolerence);
  }
}
