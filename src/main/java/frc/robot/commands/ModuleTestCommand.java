// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveModuleSubsystem;

public class ModuleTestCommand extends CommandBase {
  private SwerveModuleSubsystem m_subsystem;
  private XboxController m_controller;

  /** Creates a new ModuleTestCommand. */
  public ModuleTestCommand(SwerveModuleSubsystem subsystem, XboxController controller) {
    m_subsystem = subsystem;
    m_controller = controller;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double controllerX = m_controller.getLeftX();
    double controllerY = m_controller.getLeftY();

    SwerveModuleState desiredState = new SwerveModuleState();

    desiredState.angle = new Rotation2d(Math.atan(controllerY / controllerX));
    desiredState.speedMetersPerSecond = Math.sqrt(controllerY * controllerY + controllerX + controllerX) * 0.1;

    if (desiredState.speedMetersPerSecond > 0.02) {
      m_subsystem.setDesiredState(desiredState);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
