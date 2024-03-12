// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;

public class SwerveSubSystem extends SubsystemBase {
  SwerveDrive m_swerveDrive;
  double currentYaw;
  public double metersPSec;

  public SwerveSubSystem(SwerveDrive swerveDrive) {
    m_swerveDrive = swerveDrive;
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