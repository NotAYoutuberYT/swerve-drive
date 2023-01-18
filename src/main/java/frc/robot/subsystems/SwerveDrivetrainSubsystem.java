// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.PortConstants;

/**
 * A subsystem used to control four {@link SwerveModuleSubsystem SwerveModuleSubsystems}
 * (Requires a gyro)
 */
public class SwerveDrivetrainSubsystem extends SubsystemBase{
  private final Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
  private final Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
  private final Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
  private final Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);

  private final SwerveModuleSubsystem m_frontLeftModule;
  private final SwerveModuleSubsystem m_frontRightModule;
  private final SwerveModuleSubsystem m_rearLeftModule;
  private final SwerveModuleSubsystem m_rearRightModule;

  private final AnalogGyro m_gyro = new AnalogGyro(PortConstants.gyroPort);

  private SwerveDriveOdometry m_odometry;

  private final SwerveDriveKinematics m_kinematics =
  new SwerveDriveKinematics(
      m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  /**
   * Creates a new {@link SwerveDrivetrainSubsystem} from four {@link SwerveModuleSubsystem SwerveModuleSubsystems}
   */
  public SwerveDrivetrainSubsystem(
      SwerveModuleSubsystem frontLeftModule,
      SwerveModuleSubsystem frontRightModule,
      SwerveModuleSubsystem rearLeftModule,
      SwerveModuleSubsystem rearRighModule) {
    m_frontLeftModule = frontLeftModule;
    m_frontRightModule = frontRightModule;
    m_rearLeftModule = rearLeftModule;
    m_rearRightModule = rearRighModule;

    m_odometry = new SwerveDriveOdometry(
      m_kinematics,
      m_gyro.getRotation2d(),
      new SwerveModulePosition[] {
        m_frontLeftModule.getPosition(),
        m_frontRightModule.getPosition(),
        m_rearLeftModule.getPosition(),
        m_rearRightModule.getPosition()
      });

    m_gyro.reset();
  }

  /**
   * Method to drive the robot using speed info.
   * Speeds should be between -1 and 1, but if the robot will exceed 
   * {@link frc.robot.Constants.DriveTrainConstants#maxSpeed} it will damp the robot speed.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveTrainConstants.maxSpeed);
    m_frontLeftModule.setDesiredState(swerveModuleStates[0]);
    m_frontRightModule.setDesiredState(swerveModuleStates[1]);
    m_rearLeftModule.setDesiredState(swerveModuleStates[2]);
    m_rearRightModule.setDesiredState(swerveModuleStates[3]);
  }

  /** Updates the field relative position of the robot. */
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
