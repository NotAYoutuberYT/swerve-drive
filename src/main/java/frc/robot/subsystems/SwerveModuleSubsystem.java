// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

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

/**
 * <> A subsysem that wraps two {@link CANSparkMax CanSparkMaxes}
 * and their {@link Encoder Encoders}.
 */
public class SwerveModuleSubsystem extends SubsystemBase {
  private static final double kModuleMaxAngularVelocity = SwerveModuleConstants.maxAngularVelocity;
  private static final double kModuleMaxAngularAcceleration = SwerveModuleConstants.maxAngularAcceleration;

  private final CANSparkMax m_driveMotor;
  private final CANSparkMax m_turningMotor;

  private final AbsoluteEncoder m_driveEncoder;
  private final AbsoluteEncoder m_turningEncoder;

  /**
   * <> PID controller used for driving (p, i, and d terms are defined in
   * {@link frc.robot.Constants.SwerveModuleConstants})
   */
  private final PIDController m_drivePIDController = new PIDController(
      SwerveModuleConstants.drivingPIDControlP, SwerveModuleConstants.drivingPIDControlI,
      SwerveModuleConstants.drivingPIDControlD);
  /**
   * <> PID controller used for driving (p, i, and d terms are defined in
   * {@link frc.robot.Constants.SwerveModuleConstants})
   */
  private final ProfiledPIDController m_turningPIDController = new ProfiledPIDController(
      SwerveModuleConstants.turningPIDControllerP, SwerveModuleConstants.turningPIDControllerI,
      SwerveModuleConstants.turningPIDControllerD,
      new TrapezoidProfile.Constraints(
          kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration));

  /**
   * <> feed forward used for driving (kS and kV are defined in
   * {@link frc.robot.Constants.SwerveModuleConstants SwerveModuleConstants})
   */
  private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(
      SwerveModuleConstants.drivingFeedForwardS, SwerveModuleConstants.drivingFeedForwardV);
  /**
   * <> feed forward used for turning (kS and kV defined in
   * {@link frc.robot.Constants.SwerveModuleConstants SwerveModuleConstants})
   */
  private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(
      SwerveModuleConstants.turningFeedForwardS, SwerveModuleConstants.turningFeedForwardV);

  /**
   * <> constructs a SwerveModuleSubsystem with a drive
   * {@link CANSparkMax}, turning {@link CANSparkMax},
   * drive {@link Encoder}, and turning {@link Encoder}
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

    m_driveEncoder = m_driveMotor.getAbsoluteEncoder(Type.kDutyCycle);
    m_turningEncoder = m_turningMotor.getAbsoluteEncoder(Type.kDutyCycle);

    // <> limit the PID Controller's input range between -pi and pi and
    // set the input to be continuous
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        m_driveEncoder.getVelocity(), new Rotation2d(m_turningEncoder.getPosition()));
  }

  /**
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        m_driveEncoder.getVelocity(), new Rotation2d(m_turningEncoder.getPosition()));
  }

  /**
   * <> Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // <> optimize desired state to avoid spinning further than 90 degrees
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(m_turningEncoder.getPosition()));

    // <> calculate the drive PID controller
    final double driveOutput = m_drivePIDController.calculate(m_driveEncoder.getVelocity(), state.speedMetersPerSecond);

    // <> calculate the drive feedforward
    final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

    // <> calculate turning PID controller.
    final double turnOutput = m_turningPIDController.calculate(m_turningEncoder.getPosition(),
        state.angle.getRadians());

    // <> calculate the turn feedforward
    final double turnFeedforward = m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

    // <> update the voltages provided to the motors
    m_driveMotor.setVoltage(driveOutput + driveFeedforward);
    m_turningMotor.setVoltage(turnOutput + turnFeedforward);
  }
}
