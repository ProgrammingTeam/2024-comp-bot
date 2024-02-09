// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import org.opencv.core.Mat;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;
import frc.robot.subsystems.LimelightSub;
import frc.robot.subsystems.tankDrive;

public class LimelightDriveCom extends Command {
  /** Creates a new LimelightDriveCom. */
  private final LimelightSub m_LimelightSub;
  private final tankDrive m_TankSub;

  private double DisteanceToGo;
  private PIDController PIDCon = new PIDController(Constants.kp, 0, 0);
  public LimelightDriveCom(tankDrive TankSub, LimelightSub LimeSub) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_TankSub = TankSub;
    m_LimelightSub = LimeSub;

    addRequirements(m_TankSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_TankSub.setMotors(0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_LimelightSub.getTarget() == -1) {
      m_TankSub.setMotors(0, 0);
      return;
    }
    try {
      DisteanceToGo = m_LimelightSub.distenceFromTarget - Constants.targetDistence[m_LimelightSub.getTarget()];
    SmartDashboard.putNumber("distence to go", DisteanceToGo);
    double inverter = Math.signum(DisteanceToGo);
    m_TankSub.setMotors(Constants.DriveSpeed * inverter, Constants.DriveSpeed * inverter);
    } catch (Exception e) {
      // TODO: handle exception
    }
    
    
           
  //  SmartDashboard.putNumber("Absolute motor speed", MathUtil.clamp(Math.abs(PIDCon.calculate(DisteanceToGo)), Constants.lowDrive, Constants.highDrive));
    isFinished();
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    m_TankSub.setMotors(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(DisteanceToGo) < 3;
  }
}
