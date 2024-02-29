// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.TeleopSwerveCommand;
import frc.robot.commands.LimelightDriveCom;
import frc.robot.commands.ShootCmd;
import frc.robot.commands.ShootCmd.ShootModes;
import frc.robot.subsystems.LimelightSub;
import frc.robot.subsystems.ShooterSub;
import frc.robot.subsystems.SwerveSubSystem;
import frc.robot.commands.ClimbCom;
import frc.robot.commands.GroundIntakeCom;
import frc.robot.subsystems.ClimbSub;
import frc.robot.subsystems.GroundIntakeSub;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import java.io.File;
import edu.wpi.first.wpilibj.Filesystem;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;
import edu.wpi.first.math.util.Units;

public class RobotContainer {
  private final ShooterSub m_ShooterSub = new ShooterSub();
  private final LimelightSub m_LimelightSub = new LimelightSub();
  // The robot's subsystems and commands are defined here...
  private final GroundIntakeSub m_GroundIntakeSub = new GroundIntakeSub();
  private final ClimbSub m_ClimbSub = new ClimbSub();
  // Replace with CommandPS4Controller or CommandJoystick if needed
  public final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);
  SwerveDrive swerveDrive;
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
    swerveCommand = new TeleopSwerveCommand(swerveSubSystem, m_driverController);
    // swerveSubSystem.setDefaultCommand(swerveCommand);
    m_ClimbSub.setDefaultCommand(new ClimbCom(m_ClimbSub, .3, leftJoystick));
    configureBindings();
  }

  private void configureBindings() {
    // m_driverController.y().whileTrue(new LimelightDriveCom(swerveSubSystem,
    // m_LimelightSub));
    // m_driverController.x().whileTrue(new LimelightDriveCom(swerveSubSystem,
    // m_LimelightSub));

    m_driverController.y().whileTrue(new ShootCmd(m_ShooterSub, ShootModes.Shoot));
    m_driverController.b().whileTrue(new ShootCmd(m_ShooterSub, ShootModes.Load));
    m_driverController.x().whileTrue(new ShootCmd(m_ShooterSub, ShootModes.SpinUp));
    m_driverController.a().whileTrue(new GroundIntakeCom(m_GroundIntakeSub, 0));
    // m_driverController.x().onTrue(new ClimbCom(m_ClimbSub, .3, leftJoystick));
    // m_driverController.a().onTrue(new ClimbCom(m_ClimbSub, .3, leftJoystick));
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
