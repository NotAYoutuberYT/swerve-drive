// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveModuleConstants;

public class SwerveModuleSubsystem extends SubsystemBase {
  private static final double kModuleMaxAngularVelocity = SwerveModuleConstants.maxAngularVelocity;
  private static final double kModuleMaxAngularAcceleration = SwerveModuleConstants.maxAngularAcceleration; // radians per second squared

  private final CANSparkMax m_driveMotor;
  private final CANSparkMax m_turningMotor;

  private final Encoder m_driveEncoder;
  private final Encoder m_turningEncoder;

  /** PID controller used for driving (p, i, and d terms are defined in {@link frc.robot.Constants.SwerveModuleConstants}) */
  private final PIDController m_drivePIDController = new PIDController(
    SwerveModuleConstants.drivingPIDControlP, SwerveModuleConstants.drivingPIDControlI, SwerveModuleConstants.drivingPIDControlD);
  /** PID controller used for driving (p, i, and d terms are defined in {@link frc.robot.Constants.SwerveModuleConstants}) */
  private final ProfiledPIDController m_turningPIDController = new ProfiledPIDController(
    SwerveModuleConstants.turningPIDControllerP, SwerveModuleConstants.turningPIDControllerI, SwerveModuleConstants.turningPIDControllerD,
      new TrapezoidProfile.Constraints(
          kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration));

  /** feed forward used for driving (kS and kV are defined in {@link frc.robot.Constants.SwerveModuleConstants}) */
  private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(
    SwerveModuleConstants.drivingFeedForwardS, SwerveModuleConstants.drivingFeedForwardV);
  /** feed forward used for turning (kS and kV defined in {@link frc.robot.Constants.SwerveModuleConstants}) */
  private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(
    SwerveModuleConstants.turningFeedForwardS, SwerveModuleConstants.turningFeedForwardV);

  /**
   * Constructs a SwerveModuleSubsystem with a drive motor, turning motor, drive
   * encoder and turning encoder.ber of classes to help users implement accurate
   * feedforward control for their mechanisms. In many ways, an accurate
   * feedforward is more important than feedback to effective control of a
   * mechanism. Since most FRC® mechanisms closely obey well-understood system
   * equations, starting with an accurate feedforward is both easy and hugely
   * beneficial to accurate and robust mechanism control.
   *
   * @param driveMotorChannel      PWM output for the drive motor.
   * @param turningMotorChannel    PWM output for the turning motor.
   * @param driveEncoderChannelA   DIO input for the drive encoder channel A
   * @param driveEncoderChannelB   DIO input for the drive encoder channel B
   * @param turningEncoderChannelA DIO input for the turning encoder channel A
   * @param turningEncoderChannelB DIO input for the turning encoder channel B
   */
  public SwerveModuleSubsystem(
      int driveMotorChannel,
      int turningMotorChannel,
      int driveEncoderChannelA,
      int driveEncoderChannelB,
      int turningEncoderChannelA,
      int turningEncoderChannelB) {
    m_driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
    m_turningMotor = new CANSparkMax(turningMotorChannel, MotorType.kBrushless);

    m_driveEncoder = new Encoder(driveEncoderChannelA, driveEncoderChannelB);
    m_turningEncoder = new Encoder(turningEncoderChannelA, turningEncoderChannelB);

    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    m_driveEncoder.setDistancePerPulse(
        2 * Math.PI * SwerveModuleConstants.wheelRadius / SwerveModuleConstants.encouderResolution);

    // Set the distance (in this case, angle) in radians per pulse for the turning
    // encoder.
    // This is the the angle through an entire rotation (2 * pi) divided by the
    // encoder resolution.
    m_turningEncoder.setDistancePerPulse(2 * Math.PI / SwerveModuleConstants.encouderResolution);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        m_driveEncoder.getRate(), new Rotation2d(m_turningEncoder.getDistance()));
  }

  /**
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        m_driveEncoder.getDistance(), new Rotation2d(m_turningEncoder.getDistance()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(m_turningEncoder.getDistance()));

    // Calculate the drive output from the drive PID controller.
    final double driveOutput = m_drivePIDController.calculate(m_driveEncoder.getRate(), state.speedMetersPerSecond);

    final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput = m_turningPIDController.calculate(m_turningEncoder.getDistance(),
        state.angle.getRadians());

    final double turnFeedforward = m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

    m_driveMotor.setVoltage(driveOutput + driveFeedforward);
    m_turningMotor.setVoltage(turnOutput + turnFeedforward);
  }
}
