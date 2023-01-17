// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.DriveTrainConstants;
import frc.robot.subsystems.SwerveDrivetrainSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class DriveSwerveCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  // drive train subsystem
  private final SwerveDrivetrainSubsystem m_subsystem;

  // controller
  private final XboxController m_controller;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveSwerveCommand(SwerveDrivetrainSubsystem subsystem, XboxController controller) {
    m_subsystem = subsystem;
    m_controller = controller;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    double ySpeed = m_controller.getLeftY();
    double xSpeed = m_controller.getLeftX();
    double rot = m_controller.getRightX();

    m_subsystem.drive(xSpeed, ySpeed, rot, DriveTrainConstants.fieldRelative);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
