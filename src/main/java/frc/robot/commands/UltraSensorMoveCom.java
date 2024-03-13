// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubSystem;
import frc.robot.subsystems.UltraSonicSub;

public class UltraSensorMoveCom extends Command {
  private final UltraSonicSub m_UltraSonicSub;
  private final SwerveSubSystem m_SwerveSubSystem;
  private final double m_distanceFromObject;
  private boolean atDestination;
  /** Creates a new UltraSensorMoveCom. */
  public UltraSensorMoveCom(UltraSonicSub SonicSub, SwerveSubSystem Swerve, double DistFromObject) {
    m_UltraSonicSub = SonicSub;
    m_SwerveSubSystem = Swerve;
    m_distanceFromObject = DistFromObject;
    addRequirements(m_SwerveSubSystem, m_UltraSonicSub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   atDestination = false;  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_UltraSonicSub.getLeftRangeIn() >= m_distanceFromObject || m_UltraSonicSub.getRightRangeIn() >= m_distanceFromObject) {
      m_SwerveSubSystem.drive(0, 0.2, 0);
    }
    else {
      atDestination = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return atDestination;
  }
}
