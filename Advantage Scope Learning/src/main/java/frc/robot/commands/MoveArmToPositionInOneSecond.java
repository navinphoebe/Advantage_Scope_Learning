// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class MoveArmToPositionInOneSecond extends Command {
  /** Creates a new MoveArmToPositionInOneSecond. */
  private Arm m_arm;
  private double m_targetPosition;
  private double sign;
  public MoveArmToPositionInOneSecond(Arm arm, double targetPosition) {
    m_arm = arm;
    m_targetPosition =  targetPosition;
    sign = Math.copySign(1, m_arm.angle1 - m_targetPosition);
    addRequirements(m_arm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_arm.angle1 != m_targetPosition) {
      m_arm.angle1 -= 1 * sign;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double wristDifference = m_arm.angle1 - m_targetPosition;
    if (wristDifference % 360 == 0) {
      return true;
    }
    return false;
  }
}
