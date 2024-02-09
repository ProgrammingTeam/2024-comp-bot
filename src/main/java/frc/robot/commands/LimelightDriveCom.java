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
import frc.robot.subsystems.SwerveSubSystem;

public class LimelightDriveCom extends Command {
  /** Creates a new LimelightDriveCom. */
  private final LimelightSub m_LimelightSub;
  private final SwerveSubSystem m_SwerveSub;

  private double DisteanceToGo;
  private PIDController CenterPIDCon = new PIDController(Constants.LimelightConstants.kp, 0, 0);
  private PIDController DisteancePIDCon = new PIDController(Constants.LimelightConstants.kp, 0, 0);
  public LimelightDriveCom(SwerveSubSystem SwerveSub, LimelightSub LimeSub) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_SwerveSub = SwerveSub;
    m_LimelightSub = LimeSub;

    addRequirements(SwerveSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    CenterPIDCon.setSetpoint(0);
    CenterPIDCon.setTolerance(Constants.LimelightConstants.TxTolerance);
    DisteancePIDCon.setTolerance(Constants.LimelightConstants.TyTolerance);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_LimelightSub.getTarget() == -1) {
      m_SwerveSub.drive(0, 0, 0);
      return;
    }
    try {
      DisteancePIDCon.setSetpoint(Constants.LimelightConstants.targetDistence[m_LimelightSub.getTarget()]);
      DisteanceToGo = m_LimelightSub.distenceFromTarget - Constants.LimelightConstants.targetDistence[m_LimelightSub.getTarget()];
      SmartDashboard.putNumber("distence to go", DisteanceToGo);
      double inverter = Math.signum(DisteanceToGo);
      m_SwerveSub.drive(CenterPIDCon.calculate(m_LimelightSub.angleFromCenter()), 
        DisteancePIDCon.calculate(DisteanceToGo) * inverter, 
        Constants.LimelightConstants.targetAngle[m_LimelightSub.getTarget()]);
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
    m_SwerveSub.drive(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return CenterPIDCon.atSetpoint() && DisteancePIDCon.atSetpoint();
  }
}
