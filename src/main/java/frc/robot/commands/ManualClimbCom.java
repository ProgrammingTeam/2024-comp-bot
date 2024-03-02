// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ClimbSub;

public class ManualClimbCom extends Command {
  private final ClimbSub ClimbSub;
  private final CommandXboxController xboxController;

  public ManualClimbCom(ClimbSub manualClimbSub, CommandXboxController controller) {
    // Use addRequirements() here to declare subsystem dependencies.
      ClimbSub = manualClimbSub;
      xboxController = controller;
      addRequirements(manualClimbSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ClimbSub.setMotors(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ClimbSub.setMotors(xboxController.getLeftY());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ClimbSub.setMotors(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
