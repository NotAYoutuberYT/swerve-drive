// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.PortConstants;
import frc.robot.commands.DriveSwerveCommand;
import frc.robot.subsystems.SwerveDrivetrainSubsystem;
import frc.robot.subsystems.SwerveModuleSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // ~~ subsystems
  private final SwerveModuleSubsystem m_frontLeftSwerveModule = new SwerveModuleSubsystem(
      PortConstants.frontLeftDrive,
      PortConstants.frontLeftTurn,
      PortConstants.frontLeftDriveEncoderA,
      PortConstants.frontLeftDriveEncoderB,
      PortConstants.frontLeftTurnEncoderA,
      PortConstants.frontLeftTurnEncoderB);

  private final SwerveModuleSubsystem m_frontRightSwerveModule = new SwerveModuleSubsystem(
      PortConstants.frontRightDrive,
      PortConstants.frontRightTurn,
      PortConstants.frontRightDriveEncoderA,
      PortConstants.frontRightDriveEncoderB,
      PortConstants.frontRightTurnEncoderA,
      PortConstants.frontRightTurnEncoderB);

  private final SwerveModuleSubsystem m_rearLeftSwerveModule = new SwerveModuleSubsystem(
      PortConstants.rearLeftDrive,
      PortConstants.rearLeftTurn,
      PortConstants.rearLeftDriveEncoderA,
      PortConstants.rearLeftDriveEncoderB,
      PortConstants.rearLeftTurnEncoderA,
      PortConstants.rearLeftTurnEncoderB);

  private final SwerveModuleSubsystem m_rearRightSwerveModule = new SwerveModuleSubsystem(
      PortConstants.rearRightDrive,
      PortConstants.rearRightTurn,
      PortConstants.rearRightDriveEncoderA,
      PortConstants.rearRightDriveEncoderB,
      PortConstants.rearRightTurnEncoderA,
      PortConstants.rearRightTurnEncoderB);

  private final SwerveDrivetrainSubsystem m_SwerveDrivetrainSubsystem = new SwerveDrivetrainSubsystem(
      m_frontLeftSwerveModule,
      m_frontRightSwerveModule,
      m_rearLeftSwerveModule,
      m_rearRightSwerveModule);

  // ~~ controller
  private final XboxController m_driverController = new XboxController(ControllerConstants.driverControllerPort);

  // ~~ commands
  private final DriveSwerveCommand m_DriveSwerveCommand = new DriveSwerveCommand(
      m_SwerveDrivetrainSubsystem, m_driverController);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_SwerveDrivetrainSubsystem.setDefaultCommand(m_DriveSwerveCommand);

    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
