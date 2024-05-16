// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubSystem;

public class GlobalDriveCom extends Command {
  private final SwerveSubSystem m_SwerveSubSystem;
  private final Supplier<Double> VerticalMovement;
  private final Supplier<Double> HorizontalMovement;
  private final Supplier<Double> angularSpeed;
  private final Supplier<Double> AdjustableSpeedMultiplier;
  /** Creates a new GlobalDriveCom. */
  public GlobalDriveCom(SwerveSubSystem Swerve, Supplier<Double> VerticleSpeed, Supplier<Double> horizontalSpeed, 
                      Supplier<Double> angleSpeed, Supplier<Double> AdjSpeed) {
    m_SwerveSubSystem = Swerve;
    VerticalMovement = VerticleSpeed;
    HorizontalMovement = horizontalSpeed;
    angularSpeed = angleSpeed;
    AdjustableSpeedMultiplier = AdjSpeed;
    addRequirements(m_SwerveSubSystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_SwerveSubSystem.drive(0, 0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_SwerveSubSystem.drive(Math.pow(-HorizontalMovement.get(), 3) * AdjustableSpeedMultiplier.get(), 
                            Math.pow(-VerticalMovement.get(), 3) * AdjustableSpeedMultiplier.get(), 
                            Math.pow(-angularSpeed.get(), 3) * AdjustableSpeedMultiplier.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
