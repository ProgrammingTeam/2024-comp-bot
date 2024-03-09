// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.TeleopSwerveCommand;
import frc.robot.commands.Autos.AmpSpeakerAuto;
import frc.robot.commands.Autos.AutoSelecter;
import frc.robot.commands.Autos.DoNothing;
import frc.robot.commands.Autos.FrontSpeakerAuto;
import frc.robot.commands.Autos.SourseSpeakerAuto;
import frc.robot.commands.LimelightDriveCom;
import frc.robot.subsystems.LimelightSub;
import frc.robot.subsystems.ShooterSub;
import frc.robot.subsystems.SwerveSubSystem;
import frc.robot.subsystems.UltraSonicSub;
import frc.robot.commands.ShootCmd;
import frc.robot.commands.ShootCmd.ShootModes;
import frc.robot.commands.ButtonClimber;
import frc.robot.commands.*;
import frc.robot.commands.GroundIntakeCom;
import frc.robot.subsystems.ClimbSub;
import frc.robot.subsystems.GroundIntakeSub;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import java.io.File;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;
import edu.wpi.first.math.util.Units;

public class RobotContainer {
  private final ShooterSub m_ShooterSub = new ShooterSub();
  private final LimelightSub m_LimelightSub = new LimelightSub();
  private final GroundIntakeSub m_GroundIntakeSub = new GroundIntakeSub();
  private final ClimbSub m_ClimbSub = new ClimbSub();
  private final UltraSonicSub m_UltraSonicSub = new UltraSonicSub();
  // Replace with CommandPS4Controller or CommandJoystick if needed
  public final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);
  public final Joystick m_LeftJoystick = new Joystick(OperatorConstants.LeftJoysticPort);
  public final Joystick m_RighttJoystick = new Joystick(OperatorConstants.RighttJoysticPort);
  private final SendableChooser<AutoSelecter> autoChooser = new SendableChooser<>();

  // Swerve subsystem, command, and shooter subsystem
  SwerveDrive swerveDrive;

  public static boolean isBlueAllience() {
    return DriverStation.getAlliance().get() == Alliance.Blue;
  }
  SwerveSubSystem swerveSubSystem;
  TeleopSwerveCommand swerveCommand;
  public final CommandJoystick leftJoystick = new CommandJoystick(1);
  public final CommandJoystick RightJoystick = new CommandJoystick(2);

  public RobotContainer() {
    try {
      double maximumSpeed = Units.feetToMeters(4.5);
      File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
      swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(maximumSpeed);
    } catch (Exception e) {
      // handled exception
    }
    swerveSubSystem = new SwerveSubSystem(swerveDrive);

   swerveCommand = new TeleopSwerveCommand(swerveSubSystem, leftJoystick, RightJoystick);
    swerveSubSystem.setDefaultCommand(swerveCommand);
    autoChooser.setDefaultOption("Nothing auto", AutoSelecter.DoNothing);
    autoChooser.addOption("Front shoot auto", AutoSelecter.FrontSpeakerAuto);
    autoChooser.addOption("Sourse shoot auto", AutoSelecter.SourseSpeakerAuto);
    autoChooser.addOption("Amp shoot auto", AutoSelecter.AmpSpeakerAuto);
    SmartDashboard.putData(autoChooser);

    // swerveSubSystem.setDefaultCommand(swerveCommand);
    m_ClimbSub.setDefaultCommand(new ManualClimbCom(m_ClimbSub, m_driverController));
    configureBindings();
  }

  private void configureBindings() {
    // m_driverController.y().whileTrue(new LimelightDriveCom(swerveSubSystem,
    // m_LimelightSub));
    m_driverController.y().whileTrue(new ShootCmd(m_ShooterSub, ShootModes.Shoot));
    m_driverController.b().whileTrue(new ShootCmd(m_ShooterSub, ShootModes.Load));
    m_driverController.x().whileTrue(new ShootCmd(m_ShooterSub, ShootModes.SpinUp));
    m_driverController.a().whileTrue(new GroundIntakeCom(m_GroundIntakeSub, 0.4));

    leftJoystick.button(2).whileTrue(new ShootCmd(m_ShooterSub, ShootModes.Load));
    leftJoystick.button(1).whileTrue(new GroundIntakeCom(m_GroundIntakeSub, 0.4));
    leftJoystick.button(0).onTrue(new LimelightDriveCom(swerveSubSystem, m_LimelightSub));

    RightJoystick.button(2).onTrue(new InstantCommand(swerveSubSystem::resetGyro));
    RightJoystick.button(1).whileTrue(new ShootCmd(m_ShooterSub, ShootModes.SpinUp));
    RightJoystick.button(0).whileTrue(new ShootCmd(m_ShooterSub, ShootModes.Shoot));
    RightJoystick.button(7).whileTrue(new ButtonClimber(m_ClimbSub, 0.3));
    RightJoystick.button(13).whileTrue(new ButtonClimber(m_ClimbSub, -0.3));
  }

  public Command getAutonomousCommand() {
    switch (autoChooser.getSelected()) {
      case FrontSpeakerAuto:
        return new FrontSpeakerAuto(m_ShooterSub, swerveSubSystem, m_GroundIntakeSub);

      case SourseSpeakerAuto:
        return new SourseSpeakerAuto(m_ShooterSub, swerveSubSystem, m_LimelightSub, m_GroundIntakeSub);

      case AmpSpeakerAuto:
        return new AmpSpeakerAuto(m_ShooterSub, swerveSubSystem, m_LimelightSub, m_GroundIntakeSub);

      case DoNothing:
        return new DoNothing();
      default:
        return new DoNothing();
    }
  }
}