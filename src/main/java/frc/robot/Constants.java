// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class ControllerConstants {
    public static final int driverControllerPort = 0;
  }

  public static class SwerveModuleConstants {
    public static final double wheelRadius = 0.5;
    public static final double encouderResolution = 4096;

    public static final double maxAngularVelocity = Math.PI;
    public static final double maxAngularAcceleration = 2 * Math.PI;

    public static final double turningFeedForwardS = 1;
    public static final double turningFeedForwardV = 3;
    public static final double drivingFeedForwardS = 1;
    public static final double drivingFeedForwardV = 0.5;

    public static final double drivingPIDControlP = 1;
    public static final double drivingPIDControlI = 0;
    public static final double drivingPIDControlD = 0;

    public static final double turningPIDControllerP = 1;
    public static final double turningPIDControllerI = 0;
    public static final double turningPIDControllerD = 0;
  }

  public static class DriveTrainConstants {
    public static final boolean fieldRelative = true;
    public static final double speed = 0.1;

    // ~~ only port constant in this class
    public static final int gyroPort = -1;
  }

  public static class SwerveModulePortConstants {
    public static final int frontLeftDrive = -1;
    public static final int frontLeftTurn = -1;
    public static final int frontLeftDriveEncoderA = -1;
    public static final int frontLeftDriveEncoderB = -1;
    public static final int frontLeftTurnEncoderA = -1;
    public static final int frontLeftTurnEncoderB = -1;

    public static final int frontRightDrive = -1;
    public static final int frontRightTurn = -1;
    public static final int frontRightDriveEncoderA = -1;
    public static final int frontRightDriveEncoderB = -1;
    public static final int frontRightTurnEncoderA = -1;
    public static final int frontRightTurnEncoderB = -1;

    public static final int rearLeftDrive = -1;
    public static final int rearLeftTurn = -1;
    public static final int rearLeftDriveEncoderA = -1;
    public static final int rearLeftDriveEncoderB = -1;
    public static final int rearLeftTurnEncoderA = -1;
    public static final int rearLeftTurnEncoderB = -1;

    public static final int rearRightDrive = -1;
    public static final int rearRightTurn = -1;
    public static final int rearRightDriveEncoderA = -1;
    public static final int rearRightDriveEncoderB = -1;
    public static final int rearRightTurnEncoderA = -1;
    public static final int rearRightTurnEncoderB = -1;
  }
}
