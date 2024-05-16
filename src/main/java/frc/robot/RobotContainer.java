// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos.AmpSpeakerAuto;
import frc.robot.commands.Autos.AutoSelecter;
import frc.robot.commands.Autos.DoNothing;
import frc.robot.commands.Autos.FrontSpeakerAuto;
import frc.robot.commands.Autos.FrontSpeakerFourNoteAuto;
import frc.robot.commands.Autos.MOVEAuto;
import frc.robot.commands.Autos.SourseSpeakerAuto;
import frc.robot.commands.ShootCmd.ShootModes;
import frc.robot.subsystems.LimelightSub;
import frc.robot.subsystems.ShooterSub;
import frc.robot.subsystems.SwerveSubSystem;
import frc.robot.subsystems.UltraSonicSub;
import frc.robot.commands.*;
import frc.robot.subsystems.CameraSub;
import frc.robot.subsystems.ClimbSub;
import frc.robot.subsystems.GroundIntakeSub;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
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
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.auto.NamedCommands;

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
  private final SendableChooser<driveController> driveCon = new SendableChooser<>();
  // private final Command DriveSwitch;

  // Swerve subsystem, command, and shooter subsystem
  SwerveDrive swerveDrive;
  private final SwerveSubSystem swerveSubSystem;
  private final TeleopSwerveCommand swerveCommand;
  private final xboxTeleopSwerveCom xboxswerveCommand;
  private final GlobalDriveCom m_GlobalDriveCom;
  public static boolean isBlueAllience() {
    return DriverStation.getAlliance().get() == Alliance.Blue;
  }  

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
    m_GlobalDriveCom = new GlobalDriveCom(swerveSubSystem, () -> m_driverController.getRawAxis(0), () ->  m_driverController.getRawAxis(0), () ->  m_driverController.getRawAxis(0), () ->  m_driverController.getRawAxis(0));
    swerveCommand = new TeleopSwerveCommand(swerveSubSystem, leftJoystick, RightJoystick);
    xboxswerveCommand = new xboxTeleopSwerveCom(swerveSubSystem, m_driverController);
    swerveSubSystem.setDefaultCommand(m_GlobalDriveCom);

    // DriveSwitch = new FunctionalCommand(null, 
    // () -> {switch (driveCon.getSelected()) {

    //   case xboxControl:
    //       swerveSubSystem.setDefaultCommand(xboxswerveCommand);
    //       m_ClimbSub.setDefaultCommand(null);
    //     break;

    //   case JoystickControl: 
    //       swerveSubSystem.setDefaultCommand(swerveCommand);
    //       m_ClimbSub.setDefaultCommand(new ManualClimbCom(m_ClimbSub, m_driverController));
    //     break;

    //   default:
    //       swerveSubSystem.setDefaultCommand(xboxswerveCommand);
    //       m_ClimbSub.setDefaultCommand(null);
    //     break;}} , null, () -> false);

    NamedCommands.registerCommand("spin up", new ShootCmd(m_ShooterSub, ShootModes.SpinUp));
    NamedCommands.registerCommand("Smart Shoot", new ShootCmd(m_ShooterSub, ShootModes.SmartShoot));
    NamedCommands.registerCommand("shoot", new ShootCmd(m_ShooterSub, ShootModes.Shoot));
    NamedCommands.registerCommand("ground intake", new GroundIntakeCom(m_GroundIntakeSub, .6, 1));
    
    
    driveCon.setDefaultOption("xbox controller drive", driveController.xboxControl);
    driveCon.addOption("Joystick drive", driveController.JoystickControl);
    SmartDashboard.putData(driveCon);
    
    // swerveSubSystem = new SwerveSubSystem(swerveDrive);
    // swerveCommand = new TeleopSwerveCommand(swerveSubSystem, leftJoystick, RightJoystick);
    // xboxswerveCommand = new xboxTeleopSwerveCom(swerveSubSystem, m_driverController);
    //   switch (driveCon.getSelected()) {

    //   case xboxControl:
    //       swerveSubSystem.setDefaultCommand(xboxswerveCommand);
    //       m_ClimbSub.setDefaultCommand(null);
    //     break;

    //   case JoystickControl: 
    //       swerveSubSystem.setDefaultCommand(swerveCommand);
    //       m_ClimbSub.setDefaultCommand(new ManualClimbCom(m_ClimbSub, m_driverController));
    //     break;

    //   default:
    //       swerveSubSystem.setDefaultCommand(xboxswerveCommand);
    //       m_ClimbSub.setDefaultCommand(null);
    //     break;
    // }    

    autoChooser.setDefaultOption("Shoot auto", AutoSelecter.DoNothing);
    autoChooser.addOption("Front shoot auto", AutoSelecter.FrontSpeakerAuto);
    autoChooser.addOption("Four Note Auto", AutoSelecter.FourNoteAuto);
    autoChooser.addOption("left of subwoofer shoot auto", AutoSelecter.SourseSpeakerAuto);
    autoChooser.addOption("right of subwoofer shoot auto", AutoSelecter.AmpSpeakerAuto);
    autoChooser.addOption("MOVE backward auto", AutoSelecter.MOOOOOVE);
    autoChooser.addOption("path planner test", AutoSelecter.pathplannerTest);
    autoChooser.addOption("path planner 4 Note", AutoSelecter.PathPlannerFourNote);
    SmartDashboard.putData(autoChooser);

    // swerveSubSystem.setDefaultCommand(swerveCommand);
    //m_ClimbSub.setDefaultCommand(new ManualClimbCom(m_ClimbSub, m_driverController));
    new CameraSub();
    configureBindings();
  }

  private void configureBindings() {

   
    // m_driverController.y().whileTrue(new LimelightDriveCom(swerveSubSystem,
    // m_LimelightSub));
    m_driverController.leftBumper().whileTrue(new ButtonClimber(m_ClimbSub, 0.3));
    m_driverController.rightBumper().whileTrue(new ButtonClimber(m_ClimbSub, -0.3));
    m_driverController.x().whileTrue(new ShootCmd(m_ShooterSub, ShootModes.Shoot).alongWith(new GroundIntakeCom(m_GroundIntakeSub, 0.3, 0.25)));
    m_driverController.b().whileTrue(new ShootCmd(m_ShooterSub, ShootModes.Load));
    m_driverController.axisGreaterThan(2, 0.75).and(m_driverController.axisLessThan(3, 0.75)).whileTrue(new ShootCmd(m_ShooterSub, ShootModes.SpinUp));
    m_driverController.start().onTrue(new InstantCommand(swerveSubSystem::resetGyro, swerveSubSystem));
    //m_driverController.x().onTrue(new InstantCommand(m_ClimbSub::ResetClimbEncoders, m_ClimbSub));
    m_driverController.a().whileTrue(new GroundIntakeCom(m_GroundIntakeSub, 1,  1));
    m_driverController.y().whileTrue(new GroundIntakeCom(m_GroundIntakeSub, -0.4, -1));
    m_driverController.back().whileTrue(new ShootCmd(m_ShooterSub, ShootModes.SmartShoot).alongWith(new GroundIntakeCom(m_GroundIntakeSub, 1, 1)));

    leftJoystick.button(3).whileTrue(new ShootCmd(m_ShooterSub, ShootModes.Load));
    leftJoystick.button(2).whileTrue(new GroundIntakeCom(m_GroundIntakeSub, 1, 1));
    //leftJoystick.button(1).onTrue(new LimelightDriveCom(swerveSubSystem, m_LimelightSub));

    RightJoystick.button(3).onTrue(new InstantCommand(swerveSubSystem::resetGyro, swerveSubSystem));
    RightJoystick.button(2).whileTrue(new ShootCmd(m_ShooterSub, ShootModes.SpinUp));
    RightJoystick.button(1).whileTrue(new ShootCmd(m_ShooterSub, ShootModes.Shoot));
    RightJoystick.button(8).whileTrue(new ButtonClimber(m_ClimbSub, 0.3));
    RightJoystick.button(14).whileTrue(new ButtonClimber(m_ClimbSub, -0.3));
  }

  public enum driveController {
    JoystickControl,
    xboxControl;
  }
  
  public Command getAutonomousCommand() {
    switch (autoChooser.getSelected()) {
      case FrontSpeakerAuto:
        return new FrontSpeakerAuto(m_ShooterSub, swerveSubSystem, m_GroundIntakeSub);

      case SourseSpeakerAuto:
        return new SourseSpeakerAuto(m_ShooterSub, swerveSubSystem, m_LimelightSub, m_GroundIntakeSub, m_UltraSonicSub);

      case AmpSpeakerAuto:
        return new AmpSpeakerAuto(m_ShooterSub, swerveSubSystem, m_LimelightSub, m_GroundIntakeSub, m_UltraSonicSub);

      case FourNoteAuto:
        return new FrontSpeakerFourNoteAuto(m_ShooterSub, swerveSubSystem, m_GroundIntakeSub);

      case MOOOOOVE:
        return new MOVEAuto(swerveSubSystem);

      case DoNothing:
        return new DoNothing(m_ShooterSub, m_GroundIntakeSub);

      case pathplannerTest:
        return new PathPlannerAuto("PathPlanner 2 note test");
        
      case PathPlannerFourNote:
        return new PathPlannerAuto("PathPlanner 4 note Auto");
      default:
        return new DoNothing(m_ShooterSub, m_GroundIntakeSub);
    }
  }
}