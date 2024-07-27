// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DrivetrainSwerveDrive extends SubsystemBase {
  /** Creates a new DrivetrainSubsystem. */
  public final Field2d m_field = new Field2d();
  public double m_x;
  public double m_y;
  public double m_rotation; 
  public double fl_rotation;
  public double fr_rotation;
  public double bl_rotation;
  public double br_rotation; 
  public static final double DRIVETRAIN_TRACKWIDTH_METERS = 6;
  public static final double DRIVETRAIN_WHEELBASE_METERS = 4;
  public SwerveDriveOdometry m_odometry;
  public SwerveModuleState frontLeft;
  public SwerveModuleState frontRight;
  public SwerveModuleState backLeft;
  public SwerveModuleState backRight;
  private final StructArrayPublisher<SwerveModuleState> publisher;


   private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
          new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0), // Front left
          new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0), // Front right
          new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0), // Back left
          new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0) // Back right
  );
  

  public DrivetrainSwerveDrive() {
    // SmartDashboard.putData("Field", m_field);
    m_rotation = 0;
    fl_rotation = 0;
    fr_rotation = 0;
    bl_rotation = 0;
    br_rotation = 0;

    m_odometry = new SwerveDriveOdometry(
      m_kinematics, new Rotation2d(m_rotation),
      new SwerveModulePosition[] {
      new SwerveModulePosition(DRIVETRAIN_TRACKWIDTH_METERS, new Rotation2d(fl_rotation)),
      new SwerveModulePosition(DRIVETRAIN_TRACKWIDTH_METERS, new Rotation2d(fr_rotation)),
      new SwerveModulePosition(DRIVETRAIN_TRACKWIDTH_METERS, new Rotation2d(bl_rotation)),
      new SwerveModulePosition(DRIVETRAIN_TRACKWIDTH_METERS, new Rotation2d(br_rotation))
      }, new Pose2d(5.0, 13.5, new Rotation2d()));

    publisher = NetworkTableInstance.getDefault()
    .getStructArrayTopic("MyStates", SwerveModuleState.struct).publish();
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    ChassisSpeeds speeds = new ChassisSpeeds(1.0, 3.0, 1.5);

    SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(speeds);

    frontLeft = moduleStates[0];
    frontRight = moduleStates[1];
    backLeft = moduleStates[2];
    backRight = moduleStates[3];

    var m_pose = m_odometry.update(new Rotation2d(m_rotation),
    new SwerveModulePosition[] {
      new SwerveModulePosition(DRIVETRAIN_TRACKWIDTH_METERS, new Rotation2d(fl_rotation)),
      new SwerveModulePosition(DRIVETRAIN_TRACKWIDTH_METERS, new Rotation2d(fr_rotation)),
      new SwerveModulePosition(DRIVETRAIN_TRACKWIDTH_METERS, new Rotation2d(bl_rotation)),
      new SwerveModulePosition(DRIVETRAIN_TRACKWIDTH_METERS, new Rotation2d(br_rotation))
      });

    publisher.set(new SwerveModuleState[] {
      frontLeft,
      frontRight,
      backLeft,
      backRight
    });
  }

}
