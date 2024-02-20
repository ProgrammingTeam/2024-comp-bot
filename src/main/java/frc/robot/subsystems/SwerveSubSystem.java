// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;

public class SwerveSubSystem extends SubsystemBase {
  /** Creates a new SwerveSubSystem. */
  SwerveDrive m_swerveDrive;
  public SwerveSubSystem(SwerveDrive swerveDrive) {
    m_swerveDrive = swerveDrive;
  }

  public void drive(double Xaxis, double YAxis, double rotation){
    m_swerveDrive.drive(new Translation2d(Xaxis,YAxis),
      rotation, true, false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
