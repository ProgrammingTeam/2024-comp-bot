// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LimelightSub extends SubsystemBase {
  public  double distenceFromTarget;
  private double opposite;
  private double TxValue;
  private double TyValue;
  private int TidValue;
  private NetworkTableEntry LimeTX;
  private NetworkTableEntry LimeTY;
  private NetworkTableEntry LimeTid;

  /** Creates a new LimelightSub. */
  public LimelightSub() {
    NetworkTable Limelight = NetworkTableInstance.getDefault().getTable("limelight");
    LimeTX = Limelight.getEntry("tx");
    LimeTY = Limelight.getEntry("ty");
    LimeTid = Limelight.getEntry("tid");

  }

  @Override
  public void periodic() {
    TxValue = LimeTX.getDouble(0);
    TyValue = LimeTY.getDouble(0);
    TidValue = (int) LimeTid.getDouble(-1);

    if (TidValue >= 1 && TidValue <= 16) {
      opposite = Constants.targetHeights[TidValue] - Constants.limelightHeight;
      distenceFromTarget = opposite / Math.tan(Math.toRadians(LimeTY.getDouble(0) + Constants.angleOffset));
    }

    SmartDashboard.putNumber("Lime TX value", TxValue);
    SmartDashboard.putNumber("Lime TY value", TyValue);
    SmartDashboard.putNumber("Lime Tid value", TidValue);
    SmartDashboard.putNumber("height difference", opposite);
    SmartDashboard.putNumber("the distence from the target", distenceFromTarget);
    // This method will be called once per scheduler run
  }

  public int getTarget() {
    return (int) LimeTid.getInteger(-1);
  }
  public double angleFromCenter() {
    return TxValue;
  }
}
