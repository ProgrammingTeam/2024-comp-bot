// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.GroundIntakeCom;
import frc.robot.commands.ShootCmd;
import frc.robot.commands.ShootCmd.ShootModes;
import frc.robot.subsystems.GroundIntakeSub;
import frc.robot.subsystems.ShooterSub;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DoNothing extends SequentialCommandGroup {
  /** Creates a new DoNothing. */
  public DoNothing(ShooterSub m_ShooterSub, GroundIntakeSub m_GroundIntakeSub) {

    addCommands(
      Commands.race(
        new ShootCmd(m_ShooterSub, ShootModes.SpinUp), 
        Commands.waitSeconds(1)),
      Commands.race(
        new GroundIntakeCom(m_GroundIntakeSub, 0.3, 0.25),
        new ShootCmd(m_ShooterSub, ShootModes.Shoot), 
        Commands.waitSeconds(1))
    );
  }
}
