// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoNoteLineup;
import frc.robot.commands.AutoSwerveCommand;
import frc.robot.commands.GroundIntakeCom;
import frc.robot.commands.ShootCmd;
import frc.robot.commands.SpeakerLimLineupCom;
import frc.robot.commands.UltrasonicCmd;
import frc.robot.commands.ShootCmd.ShootModes;
import frc.robot.subsystems.GroundIntakeSub;
import frc.robot.subsystems.LimelightSub;
import frc.robot.subsystems.ShooterSub;
import frc.robot.subsystems.SwerveSubSystem;
import frc.robot.subsystems.UltraSonicSub;
import frc.robot.commands.ReverseSpeakerLimLineupCom;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SourseSpeakerAuto extends SequentialCommandGroup {
  /** Creates a new SourseSpwakerAuto. 
 * @param m_UltraSonicSub */
  public SourseSpeakerAuto(ShooterSub m_ShooterSub, SwerveSubSystem m_SwerveSub, LimelightSub m_LimelightSub, GroundIntakeSub m_GroundIntakeSub, UltraSonicSub m_UltraSonicSub) {
    addCommands(
        new ReverseSpeakerLimLineupCom(m_LimelightSub, m_SwerveSub),
        Commands.race(
            new ShootCmd(m_ShooterSub, ShootModes.SpinUp),
            Commands.waitSeconds(1)),
        Commands.race(
            new ShootCmd(m_ShooterSub, ShootModes.Shoot),
            new GroundIntakeCom(m_GroundIntakeSub, 0.35, 0.2)),
            Commands.waitSeconds(1),
        //new UltrasonicCmd(m_UltraSonicSub, m_SwerveSub),
        new AutoNoteLineup(m_SwerveSub, 45),
        Commands.race(
            new AutoSwerveCommand(m_SwerveSub, -0.25, 0, 80),
            new GroundIntakeCom(m_GroundIntakeSub, 1, 1)),
        new AutoSwerveCommand(m_SwerveSub, 0.25, 0, 85),
        Commands.race(
            new ShootCmd(m_ShooterSub, ShootModes.SpinUp),
            new ReverseSpeakerLimLineupCom(m_LimelightSub, m_SwerveSub)),
        Commands.race(
            new GroundIntakeCom(m_GroundIntakeSub, .35, 0.2),
            new ShootCmd(m_ShooterSub, ShootModes.Shoot)),
            Commands.waitSeconds(1));
  }
}
