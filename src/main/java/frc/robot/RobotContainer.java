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
import frc.robot.commands.ClimbCom;
import frc.robot.commands.GroundIntakeCom;
import frc.robot.subsystems.ClimbSub;
import frc.robot.subsystems.GroundIntakeSub;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import java.io.File;
import java.util.function.BooleanSupplier;

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
  // Subsystems
  private final LimelightSub m_LimelightSub = new LimelightSub();
  private final GroundIntakeSub m_GroundIntakeSub = new GroundIntakeSub();
  private final ClimbSub m_ClimbSub = new ClimbSub(); 
  // Hello! I wasn't looking when someone was typing

  // Controllers & Joysticks & chooser in SmartDashboard 
  public final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);
  public final Joystick m_LeftJoystick = new Joystick(OperatorConstants.LeftJoysticPort);
  public final Joystick m_RighttJoystick = new Joystick(OperatorConstants.RighttJoysticPort);
  private final SendableChooser<AutoSelecter> autoChooser = new SendableChooser<>();

  // Swerve subsystem, command, and shooter subsystem
  SwerveDrive swerveDrive;
  private final SwerveSubSystem swerveSubSystem;
  private final TeleopSwerveCommand swerveCommand;
  private final ShooterSub m_ShooterSub = new ShooterSub();

  public static boolean isBlueAllience() {
    return DriverStation.getAlliance().get() == Alliance.Blue;
  }

  public RobotContainer() {
    try {
      double maximumSpeed = Units.feetToMeters(4.5);
      File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
      swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(maximumSpeed);
    } catch (Exception e) {
      // handled exception
    }
    swerveSubSystem = new SwerveSubSystem(swerveDrive);
    swerveCommand = new TeleopSwerveCommand(swerveSubSystem, m_driverController);
    swerveSubSystem.setDefaultCommand(swerveCommand);
    autoChooser.setDefaultOption("Nothing auto", AutoSelecter.DoNothing);
    autoChooser.addOption("Front shoot auto", AutoSelecter.FrontSpeakerAuto);
    autoChooser.addOption("Sourse shoot auto", AutoSelecter.SourseSpeakerAuto);
    autoChooser.addOption("Amp shoot auto", AutoSelecter.AmpSpeakerAuto);
    SmartDashboard.putData(autoChooser);
    configureBindings();
  }

  private void configureBindings() {
    m_driverController.y().whileTrue(new LimelightDriveCom(swerveSubSystem, m_LimelightSub));
    m_driverController.b().whileTrue(new GroundIntakeCom(m_GroundIntakeSub, 0));
    m_driverController.x().whileTrue(new ClimbCom(m_ClimbSub, 0, 0));
    m_driverController.a().whileTrue(new ClimbCom(m_ClimbSub, 0, 0));
  }

  public Command getAutonomousCommand() {
    switch (autoChooser.getSelected()) {
      case FrontSpeakerAuto:
        return new FrontSpeakerAuto(m_ShooterSub, swerveSubSystem, m_GroundIntakeSub);

      case SourseSpeakerAuto:
        return new SourseSpeakerAuto(m_ShooterSub, swerveSubSystem);

      case AmpSpeakerAuto:
        return new AmpSpeakerAuto(m_ShooterSub, swerveSubSystem);

      case DoNothing:
        return new DoNothing();
      default:
        return new DoNothing();
    }
  }
}