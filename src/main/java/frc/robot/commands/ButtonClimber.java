// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSub;

public class ButtonClimber extends Command {
  private final ClimbSub climbSub;
  private double motorSpeeds;

  public ButtonClimber(ClimbSub climbSub1, double MotorSpeed) {
    climbSub = climbSub1;
    motorSpeeds = MotorSpeed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climbSub.setMotors(0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climbSub.setMotors(motorSpeeds, motorSpeeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climbSub.setMotors(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
