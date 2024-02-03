// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.LimelightSub;
import frc.robot.subsystems.tankDrive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class autoLimeDrive extends SequentialCommandGroup {
  /** Creates a new autoLimeDrive. */
  public autoLimeDrive(LimelightSub m_LimeSub, tankDrive m_TankSub) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new LimelightDriveCom(m_TankSub, m_LimeSub, 48));
  }
}
