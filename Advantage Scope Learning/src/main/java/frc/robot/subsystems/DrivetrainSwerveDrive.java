// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.GyroSim;
import frc.robot.SwerveModuleSim;

public class DrivetrainSwerveDrive extends SubsystemBase {
  /** Creates a new DrivetrainSubsystem. */
  public final Field2d m_fieldSwerve = new Field2d();
  public final GyroSim m_gyro = new GyroSim();
  public final SwerveModuleSim m_frontLeftModule = new SwerveModuleSim();
  public final SwerveModuleSim m_frontRightModule = new SwerveModuleSim();
  public final SwerveModuleSim m_backLeftModule = new SwerveModuleSim();
  public final SwerveModuleSim m_backRightModule = new SwerveModuleSim();
  public ChassisSpeeds m_speeds = new ChassisSpeeds(0, 0, 0);
  public Pose2d m_pose;
  public double m_x;
  public double m_y;
  public double m_rotationRadians; 
  public double fl_distance;
  public double fr_distance;
  public double bl_distance;
  public double br_distance; 
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
    SmartDashboard.putData("Field Swerve", m_fieldSwerve);
    m_rotationRadians = 0;
    fl_distance = 0;
    fr_distance = 0;
    bl_distance = 0;
    br_distance = 0;

    m_odometry = new SwerveDriveOdometry(
      m_kinematics, new Rotation2d(m_rotationRadians),
      new SwerveModulePosition[] {
      new SwerveModulePosition(0, new Rotation2d(0)),
      new SwerveModulePosition(0, new Rotation2d(0)),
      new SwerveModulePosition(0, new Rotation2d(0)),
      new SwerveModulePosition(0, new Rotation2d(0))
      }, new Pose2d(0, 0, new Rotation2d()));

    publisher = NetworkTableInstance.getDefault()
    .getStructArrayTopic("MyStates", SwerveModuleState.struct).publish();

    AutoBuilder.configureHolonomic(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                    1, // Max module speed, in m/s
                    0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              /* var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              } */
              return false;
            },
            this // Reference to this subsystem to set requirements
    );
    }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void resetPose(Pose2d pose) {
    SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(m_speeds);

    frontLeft = moduleStates[0];
    frontRight = moduleStates[1];
    backLeft = moduleStates[2];
    backRight = moduleStates[3];
    double frontLeftDistance = m_frontLeftModule.getValue(frontLeft.speedMetersPerSecond);
    double frontRightDistance = m_frontRightModule.getValue(frontRight.speedMetersPerSecond);
    double backLeftDistance = m_backLeftModule.getValue(backLeft.speedMetersPerSecond);
    double backRightDistance = m_backRightModule.getValue(backRight.speedMetersPerSecond);
    m_rotationRadians = m_gyro.getGyroValueAdded(m_speeds.omegaRadiansPerSecond);
    // update gyro and distance
    m_odometry.resetPosition(new Rotation2d(m_rotationRadians),
    new SwerveModulePosition[] {
      new SwerveModulePosition(frontLeftDistance, frontLeft.angle),
      new SwerveModulePosition(frontRightDistance, frontRight.angle),
      new SwerveModulePosition(backLeftDistance, backLeft.angle),
      new SwerveModulePosition(backRightDistance, backRight.angle)
      }, pose);
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return m_speeds;
  }

  public void driveRobotRelative(ChassisSpeeds speeds) {
    m_speeds = speeds;
  }

  @Override
  public void periodic() {
    SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(m_speeds);

    frontLeft = moduleStates[0];
    frontRight = moduleStates[1];
    backLeft = moduleStates[2];
    backRight = moduleStates[3];
    double frontLeftDistance = m_frontLeftModule.getValue(frontLeft.speedMetersPerSecond);
    double frontRightDistance = m_frontRightModule.getValue(frontRight.speedMetersPerSecond);
    double backLeftDistance = m_backLeftModule.getValue(backLeft.speedMetersPerSecond);
    double backRightDistance = m_backRightModule.getValue(backRight.speedMetersPerSecond);
    m_rotationRadians = m_gyro.getGyroValueAdded(m_speeds.omegaRadiansPerSecond);
    // update gyro and distance
    m_pose = m_odometry.update(new Rotation2d(m_rotationRadians),
    new SwerveModulePosition[] {
      new SwerveModulePosition(frontLeftDistance, frontLeft.angle),
      new SwerveModulePosition(frontRightDistance, frontRight.angle),
      new SwerveModulePosition(backLeftDistance, backLeft.angle),
      new SwerveModulePosition(backRightDistance, backRight.angle)
      });

    publisher.set(new SwerveModuleState[] {
      frontLeft,
      frontRight,
      backLeft,
      backRight
    });
    m_fieldSwerve.setRobotPose(m_pose); 
  }

}
