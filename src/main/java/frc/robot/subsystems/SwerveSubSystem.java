// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

public class SwerveSubSystem extends SubsystemBase {
  SwerveDrive m_swerveDrive;
  double currentYaw;
  public double metersPSec;

  public SwerveSubSystem(SwerveDrive swerveDrive) {
    m_swerveDrive = swerveDrive;
    AutoBuilder.configureHolonomic(
            m_swerveDrive::getPose, // Robot pose supplier
            m_swerveDrive::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            m_swerveDrive::getRobotVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            m_swerveDrive::drive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(2.75, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(2.75, 0.0, 0.0), // Rotation PID constants
                    0.5, // Max module speed, in m/s
                    0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );

  }

  public double getRobotOrientation() {
    return m_swerveDrive.getYaw().getDegrees();
  }

  public void drive(double XAxis, double YAxis, double rotation) {
    m_swerveDrive.drive(new Translation2d(YAxis, XAxis),
        rotation, true, false);
  }

  public void resetGyro() {
    m_swerveDrive.zeroGyro();
  }

  @Override
  public void periodic() {
    currentYaw = m_swerveDrive.getYaw().getDegrees();
    metersPSec = Math.sqrt(Math.pow(m_swerveDrive.getFieldVelocity().vxMetersPerSecond, 2) + Math.pow(m_swerveDrive.getFieldVelocity().vyMetersPerSecond, 2));
  }
}