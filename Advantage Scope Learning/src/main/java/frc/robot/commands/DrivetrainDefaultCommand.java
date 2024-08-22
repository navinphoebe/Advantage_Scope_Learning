// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.DrivetrainSubsystemSim;
import frc.robot.subsystems.DrivetrainSwerveDrive;

public class DrivetrainDefaultCommand extends Command {
  private Drivetrain m_drivetrain;
  private CommandXboxController m_driverController;
  private ChassisSpeeds _chassisSpeeds = new ChassisSpeeds(0,0,0);
  /** Creates a new DrivetrainDefaultCommand. */
  public DrivetrainDefaultCommand(Drivetrain drivetrain, CommandXboxController driverController) {
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
    double controllerDirection = DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? 1 : -1;
    double x = m_driverController.getRawAxis(1) * controllerDirection;
    double y = m_driverController.getRawAxis(0) * controllerDirection;
    double r = m_driverController.getRawAxis(2) * -1;
    x = applyDeadband(x);
    y = applyDeadband(y);
    r = applyDeadband(r);
    double maxVelocity = m_drivetrain.getMaxVelocity();
    x = x * maxVelocity;
    y = y * maxVelocity;
    r = r * Constants.DRIVE_MAX_TURN_RADIANS_PER_SECOND;

    _chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(x, y, r, m_drivetrain.getGyroscopeRotation());
    
    m_drivetrain.drive(_chassisSpeeds);
  }

  private double applyDeadband(double x) {
    if (Math.abs(x) > Constants.CONTROLLER_DEADBAND_VALUE) {
      return x;
    } else {
      return 0;
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
