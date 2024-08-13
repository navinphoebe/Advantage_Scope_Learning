// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.DrivetrainDefaultCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.DrivetrainSwerveDrive;
import frc.robot.subsystems.ExampleSubsystem;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  private final DrivetrainSwerveDrive m_swerveDrive = new DrivetrainSwerveDrive();
  private final Arm m_arm = new Arm();

  SendableChooser<Command> m_chooser = new SendableChooser<>();
  SendableChooser<Command> m_redChooser = new SendableChooser<>();
  SendableChooser<Command> m_blueChooser = new SendableChooser<>();
  SendableChooser<Command> m_positionChooser = m_redChooser;
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    m_swerveDrive.setDefaultCommand(getSwerveDriveCommand());
    
    m_driverController.b().onTrue(m_arm.initialPosition());
    m_driverController.x().onTrue(m_arm.groundPickup());
    m_driverController.y().onTrue(m_arm.defendedScoring());
    m_driverController.a().onTrue(m_arm.ampScoring());

    m_chooser.setDefaultOption("Follow Path", getFollowTestPathCommand());
    m_chooser.addOption("On the Fly", getOnTheFlyPathComamand());
    m_chooser.addOption("Navagation Grid", getNavigationGridDemoPathCommand());
    SmartDashboard.putData(m_chooser);
    
    m_redChooser.setDefaultOption("Red Amp Scoring", goToTargetPosition(13.97, 7.18, 90));
    m_redChooser.addOption("Red Speaker 1", goToTargetPosition(15.78, 6.70, 135));
    m_redChooser.addOption("Red Speaker 2", goToTargetPosition(15.20, 5.60, 180));
    m_redChooser.addOption("Red Speaker 3", goToTargetPosition(15.70, 4.35, -135));
    m_redChooser.addOption("Red Stage 1", goToTargetPosition(12.37, 5.09, -135));
    m_redChooser.addOption("Red Stage 2", goToTargetPosition(12.37, 3.05, 0));
    m_redChooser.addOption("Red Stage 3", goToTargetPosition(10.59, 4.08, 135));
    m_redChooser.addOption("Red Notes Pickup 1", goToTargetPosition(.47, 1.25, -90));
    m_redChooser.addOption("Red Notes Pickup 2", goToTargetPosition(1.04, 1.01, -90));
    m_redChooser.addOption("Red Notes Pickup 3", goToTargetPosition(1.60, .53, -90));

    m_blueChooser.setDefaultOption("Blue Amp Scoring", goToTargetPosition(1.66,7.61,90));
    m_blueChooser.addOption("Blue Speaker 1", goToTargetPosition(.75, 6.01,0));
    m_blueChooser.addOption("Blue Speaker 2", goToTargetPosition(1.45,5.56,0));
    m_blueChooser.addOption("Blue Speaker 3", goToTargetPosition(.76,4.26,0));
    m_blueChooser.addOption("Blue Stage 1", goToTargetPosition(4.17, 5.05, 0));
    m_blueChooser.addOption("Blue Stage 2", goToTargetPosition(4.28, 3.16, 0));
    m_blueChooser.addOption("Blue Stage 3", goToTargetPosition(5.98, 4.13, 0));
    m_blueChooser.addOption("Blue Notes Pickup 1", goToTargetPosition(16.09, 1.41, 0));
    m_blueChooser.addOption("Blue Notes Pickup 2", goToTargetPosition(15.52, 1.10, 0));
    m_blueChooser.addOption("Blue Notes Pickup 3", goToTargetPosition(14.85, .76, 0)); 

    SmartDashboard.putData("Target Position Command", getTargetPositionCommand());
    SmartDashboard.putData("Target Chooser", m_positionChooser);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public void setChooserToBlue() {
    m_positionChooser = m_blueChooser;
  }

  public void setChooserToRed() {
    m_positionChooser = m_redChooser;
  }
  public Command getTargetPositionCommand() {
    return m_positionChooser.getSelected();
  }

  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }

  public Command getFollowTestPathCommand() {
    // An example command will be run in autonomous
     PathPlannerPath path = PathPlannerPath.fromPathFile("New New Path");

    return AutoBuilder.followPath(path);
  } 
  public Command getOnTheFlyPathComamand() {
    List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
        new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
        new Pose2d(1.74, 7.67, Rotation2d.fromDegrees(26.57)),
        new Pose2d(14.66, 6.81, Rotation2d.fromDegrees(-36.28)),
        new Pose2d(14.95, 1.38, Rotation2d.fromDegrees(-101.98)),
        new Pose2d(0, 0, Rotation2d.fromDegrees(-26.57))
    );

    // Create the path using the bezier points created above
    PathPlannerPath path = new PathPlannerPath(
        bezierPoints,
        new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI), // The constraints for this path. If using a differential drivetrain, the angular constraints have no effect.
        new GoalEndState(0.0, Rotation2d.fromDegrees(-90)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
    );

    // Prevent the path from being flipped if the coordinates are already correct
    path.preventFlipping =true;
    return AutoBuilder.followPath(path);
  } 

  public Command getNavigationGridDemoPathCommand() { 
    Pose2d targetPose = new Pose2d(4, 4, Rotation2d.fromDegrees(180));

    // Create the constraints to use while pathfinding
    PathConstraints constraints = new PathConstraints(
            3.0, 4.0,
            Units.degreesToRadians(540), Units.degreesToRadians(720));

    // Since AutoBuilder is configured, we can use it to build pathfinding commands
      return AutoBuilder.pathfindToPose(
              targetPose,
              constraints,
              0.0, // Goal end velocity in meters/sec
              0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
    );
  }

  public Command goToTargetPosition(double x, double y, double rotationDegrees) { 
    Pose2d targetPose = new Pose2d(x, y, Rotation2d.fromDegrees(rotationDegrees));

    // Create the constraints to use while pathfinding
    PathConstraints constraints = new PathConstraints(
            3.0, 4.0,
            Units.degreesToRadians(540), Units.degreesToRadians(720));

    // Since AutoBuilder is configured, we can use it to build pathfinding commands
      return AutoBuilder.pathfindToPose(
              targetPose,
              constraints,
              0.0, // Goal end velocity in meters/sec
              0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
    );
  }

  public Command getSwerveDriveCommand() {
    return new DrivetrainDefaultCommand(
        m_swerveDrive, m_driverController);
  }
}
