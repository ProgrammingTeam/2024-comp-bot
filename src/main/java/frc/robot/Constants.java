// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int LeftJoysticPort = 1;
    public static final int RighttJoysticPort = 2;
  }

  public static class LimelightConstants {
    public static final double limelightHeight = 31;
    // Do not change zero it is a placeholder and should not be used in the code.
    public static final double[] targetHeights = { 0, 53.375, 53.375, 58.125, 58.125, 53.375, 53.375, 58.125, 58.125,
        53.375, 53.375, 52.75, 52.75, 52.75, 52.75, 52.75, 52.75 };
    public static final double[] targetDistence = { 0, 100, 60 };
    public static final double[] targetAngle = { 0, 1, 2, 180, 180, 270, 270, 180, 180, 9, 10, 300, 60, 180, 180, 60,
        300 };

    public static final double angleOffset = 0.5;

    public static final double kp = 0.002;

    public static final double lowDrive = 0.04;
    public static final double highDrive = 0.2;

    public static final double TxTolerance = 1;
    public static final double TyTolerance = 0.5;
  }

  public static final double DriveSpeed = 0.07;

  public static class GroundIntake {
    public static final int GIntakeID = 11;

    public static final int TopBeltIntakeID = 14;
    public static final int BottomBeltIntakeID = 15;
    public static final double GroundIntakeSpeed = 0.4;
  }

  public static class ShooterConstants {
    public static final double ExteriorShooterSpeed = 1;
    public static final double InteriorShooterSpeed = 0.75;
    public static final double IntakeShooterSpeed = 0.2;

    public static final int upperShooterID = 13;
    public static final int lowerShooterID = 12;
    // public static final int SeccondUpperShooterID = 14;
    public static final double DefaultShootVelocity = 0;
    public static final int IntakeLimiterSwitch = 0;
  }

  public static class sonicConstants {
    public static final int LSonicPingPort = 1;
    public static final int LSonicEchoPort = 2;
    public static final int RSonicPingPort = 3;
    public static final int RSonicEchoPort = 4;
    public static final int AmountOfInFromObject = 5;
    public static final double AutoMapSpeed = 0.25;
  }

  public static class Climb {
    public static final int LeftClimbMotorID = 10;
    public static final int RightClimbMotorID = 9;

    public static final double TopEncoderPosition = -110;
    public static final double BottomEncoderPosition = 0;
    public static final double ClimberEncoderDeadban = 0;

  }

  public static class AutoConstants {
    public static final double AutoTurnSpeed = 0;
    public static final double OrientaionOffset = 0;
    public static final double InchesTolerence = 0.5;
  }
}
