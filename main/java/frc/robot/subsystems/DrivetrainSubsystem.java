// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class DrivetrainSubsystem extends SubsystemBase {
  /** Creates a new DrivetrainSubsystem. */
  private CommandXboxController m_driverController;
  private final Field2d m_field = new Field2d();
  private double m_x;
  private double m_y;
  private double m_rotation;

  public DrivetrainSubsystem(CommandXboxController driverController) {
    SmartDashboard.putData("Field", m_field);
    m_driverController = driverController;
    m_x = 0;
    m_y = 0;
    m_rotation = 0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double x = m_driverController.getRawAxis(1);
    double y = m_driverController.getRawAxis(0);
    double rotation = m_driverController.getRawAxis(3);
    if (.05 < Math.abs(x)){
m_x += x;
    }
    if (.05 < Math.abs(y)){

    m_y += y;
    }
    if (.05 < Math.abs(rotation)){
    m_rotation += rotation;
    }
    m_field.setRobotPose(new Pose2d(m_x, m_y, new Rotation2d(m_rotation))
); 
  }

}
