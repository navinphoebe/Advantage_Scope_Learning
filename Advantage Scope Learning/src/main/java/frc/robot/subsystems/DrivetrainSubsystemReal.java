// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.MechanicalConfiguration;
import com.swervedrivespecialties.swervelib.MkModuleConfiguration;
import com.swervedrivespecialties.swervelib.MkSwerveModuleBuilder;
import com.swervedrivespecialties.swervelib.MotorType;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

import static frc.robot.subsystems.DrivetrainSubsystemConstants.*;

import java.io.Console;

public class DrivetrainSubsystemReal extends SubsystemBase implements Drivetrain {
  public static final MechanicalConfiguration MK4I_L2_PLUS = new MechanicalConfiguration(
    0.10033,
    (16.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0),
    true,
    (16.0 / 50.0) * (10.0 / 60.0),
    false
);


  /**
   * The maximum voltage that will be delivered to the drive motors.
   * This can be reduced to cap the robot's maximum speed. Typically, this is useful during initial testing of the robot.
   */
  public static final double MAX_VOLTAGE = 12.0;
  // Measure the drivetrain's maximum velocity or calculate the theoretical.
  // The formula for calculating the theoretical maximum velocity is:
  // <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> * pi
  // By default this value is setup for a Mk3 standard module using Falcon500s to drive.
  // An example of this constant for a Mk4 L2 module with NEOs to drive is:
  // 5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() * SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI
  /**
   * The maximum velocity of the robot in meters per second.
   * This is a measure of how fast the robot should be able to drive in a straight line.
   */
  public static final double MAX_VELOCITY_METERS_PER_SECOND = 5880 / 60 *
          SdsModuleConfigurations.MK4I_L2.getDriveReduction() *
          SdsModuleConfigurations.MK4I_L2.getWheelDiameter() * Math.PI;
  
  /**
   * The maximum angular velocity of the robot in radians per second.
   * This is a measure of how fast the robot can rotate in place.
   */
  // Here we calculate the theoretical maximum angular velocity. You can also replace this with a measured amount.
  public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
          Math.hypot(DrivetrainSubsystemConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DrivetrainSubsystemConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0);

  public final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
          new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0), // Front left
          new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0), // Front right
          new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0), // Back left
          new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0) // Back right
  );
  
  private final AHRS m_navx = new AHRS(SPI.Port.kMXP, (byte) 200);

  private Pose2d _targetPose = new Pose2d(Units.feetToMeters(1.54), Units.feetToMeters(23.23), Rotation2d.fromDegrees(-180));

  public final SwerveModule m_frontLeftModule;
  public final SwerveModule m_frontRightModule;
  public final SwerveModule m_backLeftModule;
  public final SwerveModule m_backRightModule;
  private final SwerveDriveOdometry  _odometryFromHardware;
  private SwerveModuleState[] _moduleStates = m_kinematics.toSwerveModuleStates(new ChassisSpeeds(0,0,0));

  StructArrayPublisher<SwerveModuleState> _swervePublisher = NetworkTableInstance.getDefault()
    .getStructArrayTopic("SmartDashboard/SwerveModuleStates", SwerveModuleState.struct).publish();

  private Field2d _field2d = new Field2d();

  /** Creates a new DrivetrainSubsystem. */
  public DrivetrainSubsystemReal() {
        MkModuleConfiguration moduleConfig = new MkModuleConfiguration();
      moduleConfig.setSteerCurrentLimit(30.0);
      moduleConfig.setDriveCurrentLimit(40.0);
      moduleConfig.setSteerPID(0.2, 0.0, 0.1);

    // SmartDashboard.putNumber("GearRatio L1 wheel diameter", GearRatio.L1.getConfiguration().getWheelDiameter());
    // SmartDashboard.putNumber("GearRatio L1 drive reduction", GearRatio.L1.getConfiguration().getDriveReduction());

    m_frontLeftModule = new MkSwerveModuleBuilder(moduleConfig)
      .withGearRatio(MK4I_L2_PLUS)
      .withDriveMotor(MotorType.NEO, FRONT_LEFT_MODULE_DRIVE_MOTOR)
      .withSteerMotor(MotorType.NEO, FRONT_LEFT_MODULE_STEER_MOTOR)
      .withSteerEncoderPort(FRONT_LEFT_MODULE_STEER_ENCODER)
      .withSteerOffset(FRONT_LEFT_MODULE_STEER_OFFSET)
      .build();

    m_frontRightModule = new MkSwerveModuleBuilder(moduleConfig)
      .withGearRatio(MK4I_L2_PLUS)
      .withDriveMotor(MotorType.NEO, FRONT_RIGHT_MODULE_DRIVE_MOTOR)
      .withSteerMotor(MotorType.NEO, FRONT_RIGHT_MODULE_STEER_MOTOR)
      .withSteerEncoderPort(FRONT_RIGHT_MODULE_STEER_ENCODER)
      .withSteerOffset(FRONT_RIGHT_MODULE_STEER_OFFSET)
      .build();

    m_backLeftModule = new MkSwerveModuleBuilder(moduleConfig)
      .withGearRatio(MK4I_L2_PLUS)
      .withDriveMotor(MotorType.NEO, BACK_LEFT_MODULE_DRIVE_MOTOR)
      .withSteerMotor(MotorType.NEO, BACK_LEFT_MODULE_STEER_MOTOR)
      .withSteerEncoderPort(BACK_LEFT_MODULE_STEER_ENCODER)
      .withSteerOffset(BACK_LEFT_MODULE_STEER_OFFSET)
      .build();

    m_backRightModule = new MkSwerveModuleBuilder(moduleConfig)
      .withGearRatio(MK4I_L2_PLUS)
      .withDriveMotor(MotorType.NEO, BACK_RIGHT_MODULE_DRIVE_MOTOR)
      .withSteerMotor(MotorType.NEO, BACK_RIGHT_MODULE_STEER_MOTOR)
      .withSteerEncoderPort(BACK_RIGHT_MODULE_STEER_ENCODER)
      .withSteerOffset(BACK_RIGHT_MODULE_STEER_OFFSET)
      .build();

    double swerveDriveDelay = 0;
    double swerveRotateDelay = 0.25;
    ((CANSparkMax) m_frontLeftModule.getSteerMotor()).setOpenLoopRampRate(swerveRotateDelay);
    ((CANSparkMax) m_frontRightModule.getSteerMotor()).setOpenLoopRampRate(swerveRotateDelay);
    ((CANSparkMax) m_backLeftModule.getSteerMotor()).setOpenLoopRampRate(swerveRotateDelay);
    ((CANSparkMax) m_backRightModule.getSteerMotor()).setOpenLoopRampRate(swerveRotateDelay);

    ((CANSparkMax) m_frontLeftModule.getDriveMotor()).setOpenLoopRampRate(swerveDriveDelay); 
    ((CANSparkMax) m_frontRightModule.getDriveMotor()).setOpenLoopRampRate(swerveDriveDelay);
    ((CANSparkMax) m_backLeftModule.getDriveMotor()).setOpenLoopRampRate(swerveDriveDelay);
    ((CANSparkMax) m_backRightModule.getDriveMotor()).setOpenLoopRampRate(swerveDriveDelay);

    /*_odometryFromKinematics = new SwerveDriveOdometry(m_kinematics, this.getGyroscopeRotation(), 
    new SwerveModulePosition[] {
      m_frontLeftModule.getPosition(),
      m_frontRightModule.getPosition(),
      m_backLeftModule.getPosition(),
      m_backRightModule.getPosition()
    }, new Pose2d(0, 0, new Rotation2d()));*/

    _odometryFromHardware = new SwerveDriveOdometry(
      m_kinematics, this.getGyroscopeRotation(), 
      new SwerveModulePosition[] {
        m_frontLeftModule.getPosition(),
        m_frontRightModule.getPosition(),
        m_backLeftModule.getPosition(),
        m_backRightModule.getPosition()
      }, new Pose2d(0, 0, new Rotation2d()));
    // _diagnostics = new DrivetrainDiagnosticsShuffleboard();
    // _driveCharacteristics = new DriveCharacteristics();

    SmartDashboard.putData("HardwareOdometry Field", _field2d);
  }

  @Override
  public void drive(ChassisSpeeds targetChassisSpeeds) {
    _moduleStates = m_kinematics.toSwerveModuleStates(targetChassisSpeeds);
  }

  @Override
  public void resetPose(Pose2d pose) {
    _odometryFromHardware.resetPosition(
      this.getGyroscopeRotation(), 
      new SwerveModulePosition[] {
        m_frontLeftModule.getPosition(),
        m_frontRightModule.getPosition(),
        m_backLeftModule.getPosition(),
        m_backRightModule.getPosition()
      }, pose);
  }

  @Override 
  public void setDisabled() {
    _moduleStates = m_kinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0, 0));
  }

  @Override
  public void periodic() {
    SwerveModuleState[] states = _moduleStates; // states and _modulestates still point to the same data
    SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);

    SmartDashboard.putNumber("FL encoder", Math.toDegrees(m_frontLeftModule.getSteerEncoder().getAbsoluteAngle()));
    SmartDashboard.putNumber("FR encoder", Math.toDegrees(m_frontRightModule.getSteerEncoder().getAbsoluteAngle()));
    SmartDashboard.putNumber("BL encoder", Math.toDegrees(m_backLeftModule.getSteerEncoder().getAbsoluteAngle()));
    SmartDashboard.putNumber("BR encoder", Math.toDegrees(m_backRightModule.getSteerEncoder().getAbsoluteAngle()));

    _odometryFromHardware.update(
      this.getGyroscopeRotation(), 
      new SwerveModulePosition[] {
        m_frontLeftModule.getPosition(),
        m_frontRightModule.getPosition(),
        m_backLeftModule.getPosition(),
        m_backRightModule.getPosition()
      });

    m_frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[0].angle.getRadians());
    m_frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[1].angle.getRadians());
    m_backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[2].angle.getRadians());
    m_backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[3].angle.getRadians());
    
    _field2d.setRobotPose(_odometryFromHardware.getPoseMeters());

    SwerveModuleState[] hardwareStates = new SwerveModuleState[4];
    hardwareStates[0] = ToSwerveModuleState(m_frontLeftModule);
    hardwareStates[1] = ToSwerveModuleState(m_frontRightModule);
    hardwareStates[2] = ToSwerveModuleState(m_backLeftModule);
    hardwareStates[3] = ToSwerveModuleState(m_backRightModule);

    _swervePublisher.set(states);
  }

  private static SwerveModuleState ToSwerveModuleState(SwerveModule module) {
    return new SwerveModuleState(
      module.getDriveVelocity(),
      new Rotation2d(module.getSteerEncoder().getAbsoluteAngle()));
  } 

  public Rotation2d getGyroscopeRotation() {
    if (m_navx.isMagnetometerCalibrated()) {
      // We will only get valid fused headings if the magnetometer is calibrated
      return Rotation2d.fromDegrees(m_navx.getFusedHeading());
    }

    // We have to invert the angle of the NavX so that rotating the robot counter-clockwise makes the angle increase.
    return Rotation2d.fromDegrees(360.0 - m_navx.getAngle());
  }

  public void zeroGyroscope() {
    var hardwarePose = _odometryFromHardware.getPoseMeters();
    _odometryFromHardware.resetPosition(
      this.getGyroscopeRotation(), 
      new SwerveModulePosition[] {
        m_frontLeftModule.getPosition(),
        m_frontRightModule.getPosition(),
        m_backLeftModule.getPosition(),
        m_backRightModule.getPosition()
      }, new Pose2d(hardwarePose.getTranslation(), new Rotation2d()));
  }

  public Rotation2d getOdometryRotation() {
    return _odometryFromHardware.getPoseMeters().getRotation();
  }
}