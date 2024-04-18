// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoSwerveCommand;
import frc.robot.commands.GroundIntakeCom;
import frc.robot.commands.ShootCmd;
import frc.robot.commands.ShootCmd.ShootModes;
import frc.robot.subsystems.GroundIntakeSub;
import frc.robot.subsystems.ShooterSub;
import frc.robot.subsystems.SwerveSubSystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FrontSpeakerFourNoteAuto extends SequentialCommandGroup {
  /** Creates a new FrontSpeakerAuto. */
  private final double TimeToShoot = 0.4;
  private final double DrivePercentage = 0.25;
  private final double noteSeparation = 57;
  private final double SpeakerToNoteDist = 78;
  private final double LongXPercentage = noteSeparation/SpeakerToNoteDist;
  private final double DistAwayFromNote = 36;
  private final double DistReverseDiagnal = SpeakerToNoteDist - DistAwayFromNote;
  private final double ShortXPercentage = noteSeparation/DistReverseDiagnal;
  private final double ShortDiagDistMagniutude = Math.sqrt(DistReverseDiagnal*DistReverseDiagnal + noteSeparation*noteSeparation);
  private final double LongDiagDistMagniutude = Math.sqrt(SpeakerToNoteDist*SpeakerToNoteDist + noteSeparation*noteSeparation);
  
  public FrontSpeakerFourNoteAuto(ShooterSub m_ShooterSub, SwerveSubSystem m_SwerveSub, GroundIntakeSub m_GroundIntakeSub) {
    addCommands(
      //first loaded note and behind note to shoot
        Commands.race(
            new ShootCmd(m_ShooterSub, ShootModes.SpinUp),
            Commands.waitSeconds(1)),
        Commands.race(
            new ShootCmd(m_ShooterSub, ShootModes.Shoot),
            new GroundIntakeCom(m_GroundIntakeSub, .35, 0.2),
            Commands.waitSeconds(TimeToShoot)),
        Commands.race(
            new AutoSwerveCommand(m_SwerveSub, -DrivePercentage, 0, 80),
            new GroundIntakeCom(m_GroundIntakeSub, .6, 1.0)),
        Commands.race(
            new AutoSwerveCommand(m_SwerveSub, DrivePercentage, 0, 85),
            new ShootCmd(m_ShooterSub, ShootModes.SpinUp),
            new GroundIntakeCom(m_GroundIntakeSub, .6, 1)),
        Commands.race(
            new ShootCmd(m_ShooterSub, ShootModes.Shoot),
            Commands.waitSeconds(TimeToShoot)),
      //Left speaker note collect and shoot
        new AutoSwerveCommand(m_SwerveSub, -DrivePercentage, ShortXPercentage * DrivePercentage, ShortDiagDistMagniutude),
        Commands.race(
            new AutoSwerveCommand(m_SwerveSub, -DrivePercentage, 0, DistAwayFromNote),
            new GroundIntakeCom(m_GroundIntakeSub, 0.6, 1)),
        Commands.race(
            new ShootCmd(m_ShooterSub, ShootModes.SpinUp),
            new AutoSwerveCommand(m_SwerveSub, DrivePercentage, -LongXPercentage * DrivePercentage, LongDiagDistMagniutude)),
        Commands.race(
            new ShootCmd(m_ShooterSub, ShootModes.Shoot),
            new GroundIntakeCom(m_GroundIntakeSub, .35, 0.2),
            Commands.waitSeconds(TimeToShoot)),
      //Right speaker note collect and shoot
        new AutoSwerveCommand(m_SwerveSub, -DrivePercentage, -ShortXPercentage * DrivePercentage, ShortDiagDistMagniutude),
        Commands.race(
            new AutoSwerveCommand(m_SwerveSub, -DrivePercentage, 0, DistAwayFromNote),
            new GroundIntakeCom(m_GroundIntakeSub, 0.6, 1)),
        Commands.race(
            new ShootCmd(m_ShooterSub, ShootModes.SpinUp),
            new AutoSwerveCommand(m_SwerveSub, DrivePercentage, LongXPercentage * DrivePercentage, LongDiagDistMagniutude)),
        Commands.race(
            new ShootCmd(m_ShooterSub, ShootModes.Shoot),
            new GroundIntakeCom(m_GroundIntakeSub, .35, 0.2),
            Commands.waitSeconds(TimeToShoot)));
       // new AutoSwerveCommand(m_SwerveSub, 0.5, 0, 12));
  }

}
