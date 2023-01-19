// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Arrays;
import java.util.Collections;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.PortConstants;

/**
 * <> A subsystem used to control four {@link SwerveModuleSubsystem SwerveModuleSubsystems}
 * (Requires an {@link AnalogGyro})
 */
public class SwerveDriveTrainSubsystem extends SubsystemBase{
  // <> position of the wheels (0, 0 is center of robot)
  // the way wpilib does this is weird so if you have any
  // questions about this ask me :)
  private final Translation2d m_frontLeftLocation = new Translation2d(
    DriveTrainConstants.driveTrainLength1 / 2, DriveTrainConstants.driveTrainLength2 / 2);
  private final Translation2d m_frontRightLocation = new Translation2d(
    DriveTrainConstants.driveTrainLength1 / 2, -DriveTrainConstants.driveTrainLength2 / 2);
  private final Translation2d m_backLeftLocation = new Translation2d(
    -DriveTrainConstants.driveTrainLength1 / 2, DriveTrainConstants.driveTrainLength2 / 2);
  private final Translation2d m_backRightLocation = new Translation2d(
    DriveTrainConstants.driveTrainLength1 / 2, DriveTrainConstants.driveTrainLength2 / 2);

  private final SwerveModuleSubsystem m_frontLeftModule;
  private final SwerveModuleSubsystem m_frontRightModule;
  private final SwerveModuleSubsystem m_rearLeftModule;
  private final SwerveModuleSubsystem m_rearRightModule;

  // <> this is used for field-relative control
  private final AnalogGyro m_gyro = new AnalogGyro(PortConstants.gyroPort);

  private SwerveDriveOdometry m_odometry;

  private final SwerveDriveKinematics m_kinematics =
  new SwerveDriveKinematics(
      m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  /**
   * <> Creates a new {@link SwerveDriveTrainSubsystem} from four {@link SwerveModuleSubsystem SwerveModuleSubsystems}
   */
  public SwerveDriveTrainSubsystem(
      SwerveModuleSubsystem frontLeftModule,
      SwerveModuleSubsystem frontRightModule,
      SwerveModuleSubsystem rearLeftModule,
      SwerveModuleSubsystem rearRighModule) {
    m_frontLeftModule = frontLeftModule;
    m_frontRightModule = frontRightModule;
    m_rearLeftModule = rearLeftModule;
    m_rearRightModule = rearRighModule;

    m_gyro.reset();

    m_odometry = new SwerveDriveOdometry(
      m_kinematics,
      m_gyro.getRotation2d(),
      new SwerveModulePosition[] {
        m_frontLeftModule.getPosition(),
        m_frontRightModule.getPosition(),
        m_rearLeftModule.getPosition(),
        m_rearRightModule.getPosition()
      });
  }

  /**
   * <> drives the robot using speed and rotation info
   * (same controls as a {@link edu.wpi.first.wpilibj.drive.MecanumDrive MecanumDrive}).
   * speeds should be between -1 and 1, but if the robot will exceed 
   * {@link frc.robot.Constants.DriveTrainConstants#maxSpeed} it will damp the  speed.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // <> convert the inputted values into swerveModuleStates
    SwerveModuleState[] swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    
    // <> ensure the robot doesn't exceed the max speed
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveTrainConstants.maxSpeed);

    // <> if the robot is told to drive a miniscule ammount, don't
    // rotate the wheels
    double robotSpeed = Collections.max(Arrays.asList(swerveModuleStates)).speedMetersPerSecond;
    if (robotSpeed < DriveTrainConstants.minSpeed) {
      return;
    }

    // <> update swerve modules
    m_frontLeftModule.setDesiredState(swerveModuleStates[0]);
    m_frontRightModule.setDesiredState(swerveModuleStates[1]);
    m_rearLeftModule.setDesiredState(swerveModuleStates[2]);
    m_rearRightModule.setDesiredState(swerveModuleStates[3]);
  }

  /** <> updates the field relative position of the robot */
  public void updateOdometry() {
    m_odometry.update(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
          m_frontLeftModule.getPosition(),
          m_frontRightModule.getPosition(),
          m_rearLeftModule.getPosition(),
          m_rearRightModule.getPosition()
        });
  }
}
