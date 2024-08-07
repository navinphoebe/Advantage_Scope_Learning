// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;
import frc.robot.subsystems.DrivetrainSwerveDrive;

public class DrivetrainDefaultCommand extends Command {
  private DrivetrainSwerveDrive m_drivetrain;
  private CommandXboxController m_driverController;
  private ChassisSpeeds _chassisSpeeds = new ChassisSpeeds(0,0,0);
  /** Creates a new DrivetrainDefaultCommand. */
  public DrivetrainDefaultCommand(DrivetrainSwerveDrive drivetrain, CommandXboxController driverController) {
    m_driverController = driverController;
    m_drivetrain = drivetrain;
    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _chassisSpeeds = new ChassisSpeeds(0, 0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    _chassisSpeeds = new ChassisSpeeds(m_driverController.getLeftX(), m_driverController.getLeftY(), m_driverController.getRightX());
    m_drivetrain.driveRobotRelative(_chassisSpeeds);
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
