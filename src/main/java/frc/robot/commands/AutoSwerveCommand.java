// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubSystem;

public class AutoSwerveCommand extends Command {
  private final SwerveSubSystem m_swerveSubSystem;
  private final double m_YMovement;
  private final double m_XMovement;
  private final double m_distanceNeeded;
  private  double finalOrientation;
  private double distanceTraveled;
  private double turnSpeed;
  private final boolean useTurning;
  

  public AutoSwerveCommand(SwerveSubSystem swerveSubSystem, double YMove, double XMove, double distance) {
    m_swerveSubSystem = swerveSubSystem;
    m_YMovement = YMove;
    m_XMovement = XMove;
    m_distanceNeeded = distance;
    useTurning = false;
    finalOrientation = m_swerveSubSystem.getRobotOrientation();
    addRequirements(m_swerveSubSystem);
  }

    public AutoSwerveCommand(SwerveSubSystem swerveSubSystem, double YMove, double XMove, double distance, double angle) {
    m_swerveSubSystem = swerveSubSystem;
    m_YMovement = YMove;
    m_XMovement = XMove;
    m_distanceNeeded = distance;
    finalOrientation = angle;
    useTurning = true;
    addRequirements(m_swerveSubSystem);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (!useTurning) finalOrientation = m_swerveSubSystem.getRobotOrientation(); 
    
    distanceTraveled = 0;
    m_swerveSubSystem.drive(0, 0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Rotation value subject to change
    if(MathUtil.isNear(finalOrientation, m_swerveSubSystem.getRobotOrientation(), 3)) {
      turnSpeed = 0;
    }
    else if (m_swerveSubSystem.getRobotOrientation() <= finalOrientation) {
      turnSpeed = Constants.AutoConstants.AutoTurnSpeed;
    } 
    else if(m_swerveSubSystem.getRobotOrientation() >= finalOrientation){
      turnSpeed = -Constants.AutoConstants.AutoTurnSpeed;
    } 

    m_swerveSubSystem.drive(m_XMovement, m_YMovement, turnSpeed);
    distanceTraveled += Units.metersToInches(m_swerveSubSystem.metersPSec / 50);
    SmartDashboard.putNumber("Preceived Distence Traveled", distanceTraveled);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerveSubSystem.drive(0, 0, 0);
    distanceTraveled = 0;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_distanceNeeded  <= distanceTraveled;
  }
}
