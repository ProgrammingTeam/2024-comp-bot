// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightSub;
import frc.robot.subsystems.SwerveSubSystem;

public class VisionPoseCom extends Command {
  private final SwerveSubSystem m_Drive;
  private final LimelightSub m_Limelight;
  private double YSpeed;
  private double XSpeed;
  private double Rotation;
  private boolean XTarget;
  private boolean YTarget;
  private boolean RotTarget;

  /** Creates a new VisonPoseCom. */
  public VisionPoseCom(SwerveSubSystem Drive, LimelightSub Limelight) {
    m_Drive = Drive;
    m_Limelight = Limelight; 
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //m_Limelight.getPose();
    m_Drive.drive(0, 0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  if(m_Limelight.distenceFromTarget > 19) {
  YSpeed = -0.4;
}
else if(m_Limelight.distenceFromTarget < 21) {
  YSpeed = 0.4;
}
else {
  YSpeed = 0;
  YTarget = true;
}
  if(m_Limelight.VerticleOffsetFromTarget > 19) {
  XSpeed = -0.4;
}
else if(m_Limelight.VerticleOffsetFromTarget < 21) {
  XSpeed = 0.4;
}
else {
  XSpeed = 0;
  XTarget = true;
}
  if(m_Limelight.angleFromCenter() > 1) {
  Rotation = -0.4;
}
else if(m_Limelight.angleFromCenter() < -1) {
  Rotation = 0.4;
}
else {
  Rotation = 0;
  RotTarget = true;
}
    m_Drive.drive(XSpeed, YSpeed, Rotation);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return XTarget&&YTarget&&RotTarget;
  }
}
