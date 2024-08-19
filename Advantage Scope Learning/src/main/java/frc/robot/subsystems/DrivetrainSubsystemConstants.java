package frc.robot.subsystems;

public class DrivetrainSubsystemConstants {
      // DRIVETRAIN
    // start at 10
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.4445; // The left-to-right distance between the drivetrain wheels
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.5461; // The front-to-back distance between the drivetrain wheels.

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 16; // Updated 
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 15; // Updated
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 21; // Updated
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = Math.toRadians(214); // competition bot
    // public static final double FRONT_LEFT_MODULE_STEER_OFFSET = Math.toRadians(220); // kitbot

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 17; // Updated
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 11; // Updated
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 24; // Updated
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = Math.toRadians(399); // competition bot
    // public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = Math.toRadians(35); // kitbot

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 10; // Updated
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 14; // Updated
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 22; // Updated
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = Math.toRadians(229); // competition bot
    // public static final double BACK_LEFT_MODULE_STEER_OFFSET = Math.toRadians(225); // kitbot

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 13; // Updated
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 12; // Updated
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 23; // Updated
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = Math.toRadians(168); // competition bot
    // public static final double BACK_RIGHT_MODULE_STEER_OFFSET = Math.toRadians(170); // kitbot
}