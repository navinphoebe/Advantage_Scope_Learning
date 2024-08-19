// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface Drivetrain extends Subsystem {
  
  public void drive(ChassisSpeeds speeds);

  public Rotation2d getGyroscopeRotation();

  public void resetPose(Pose2d pose);

  public void setDisabled();
}
