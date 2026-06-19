package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.subsystems.SwerveModuleConfig;

public class SwerveConstants {
    public static final double MAX_SPEED_METERS_PER_SECOND = 0;
    public static final double MAX_ANGULAR_SPEED = Double.MAX_VALUE;

    public static final double DIRECTION_SLEW_RATE = 0;
    public static final double MAGNITUDE_SLEW_RATE = 0;
    public static final double ROTATIONAL_SLEW_RATE = 0;

    public static final double TRACK_WIDTH = 0;
    public static final double WHEEL_BASE = 0;

    public static final SwerveDriveKinematics DRIVE_KINEMATICS =
            new SwerveDriveKinematics(
                    new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
                    new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
                    new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
                    new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2));

    public static final double FRONT_LEFT_CHASSIS_ANGULAR_OFFSET = 0;
    public static final double FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET = 0;
    public static final double REAR_LEFT_CHASSIS_ANGULAR_OFFSET = 0;
    public static final double REAR_RIGHT_CHASSIS_ANGULAR_OFFSET = 0;

    public static final int FRONT_LEFT_DRIVING_CAN_ID = 0;
    public static final int FRONT_RIGHT_DRIVING_CAN_ID = 0;
    public static final int REAR_LEFT_DRIVING_CAN_ID = 0;
    public static final int REAR_RIGHT_DRIVING_CAN_ID = 0;

    public static final int FRONT_LEFT_TURNING_CAN_ID = 0;
    public static final int FRONT_RIGHT_TURNING_CAN_ID = 0;
    public static final int REAR_LEFT_TURNING_CAN_ID = 0;
    public static final int REAR_RIGHT_TURNING_CAN_ID = 0;

    public static final boolean GYRO_REVERSED = false;

    public static final int DRIVING_MOTOR_PINION_TEETH = 0;
    public static final int BEVEL_GEAR_TEETH = 0;
    public static final int FIRST_STAGE_SPUR_GEAR_TEETH = 0;
    public static final int BEVEL_PINION_TEETH = 0;

    public static final int DRIVING_MOTOR_FREE_SPEED = 0;

    public static final double DRIVING_MOTOR_FREE_SPEED_RPS =
            (double) DRIVING_MOTOR_FREE_SPEED / 60;
    public static final double WHEEL_DIAMETER_METERS = 0;
    public static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;
    public static final double DRIVING_MOTOR_REDUCTION =
            (double) (BEVEL_GEAR_TEETH * FIRST_STAGE_SPUR_GEAR_TEETH)
                    / (DRIVING_MOTOR_PINION_TEETH * BEVEL_PINION_TEETH);
    public static final double DRIVE_WHEEL_FREE_SPEED_RPS =
            (DRIVING_MOTOR_FREE_SPEED_RPS * WHEEL_CIRCUMFERENCE_METERS) / DRIVING_MOTOR_REDUCTION;

    public static final double ROBOT_MASS_KG = 0;
    public static final double ROBOT_MOI = 0;

    public static final double WHEEL_RADIUS_METERS = 0;
    public static final double MAX_DRIVE_VELOCITY_MPS = 4;
    public static final double WHEEL_COF = 1.0;
    public static final DCMotor DRIVE_MOTOR = new DCMotor(0, 0, 0, 0, 0, 0);
    public static final double DRIVE_CURRENT_LIMIT = 40;
    public static final int NUM_MOTORS = 1;
}
