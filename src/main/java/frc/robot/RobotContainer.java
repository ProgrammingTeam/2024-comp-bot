// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.MotorFeedbackSensor;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.TeleopSwerveCommand;
import frc.robot.commands.AutonomousCmds.AutoDriveGroup;
import frc.robot.subsystems.SwerveSubSystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import java.io.File;
import edu.wpi.first.wpilibj.Filesystem;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class RobotContainer {

  public final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);
  SwerveDrive swerveDrive;
  SwerveSubSystem swerveSubSystem = new SwerveSubSystem(swerveDrive);
  TeleopSwerveCommand swerveCommand = new TeleopSwerveCommand(swerveSubSystem, m_driverController);

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
    configureBindings();
  }

  private void configureBindings() {

    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  public Command getAutonomousCommand() {
    return new AutoDriveGroup(swerveSubSystem);
  }
}
