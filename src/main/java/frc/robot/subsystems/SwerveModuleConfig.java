package frc.robot.subsystems;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.constants.SwerveConstants;

public class SwerveModuleConfig {
    public static final SparkFlexConfig drivingConfig = new SparkFlexConfig();
    public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

    static {
        double drivingFactor =
                SwerveConstants.WHEEL_DIAMETER_METERS
                        * Math.PI
                        / SwerveConstants.DRIVING_MOTOR_REDUCTION;
        double turningFactor = 2 * Math.PI;

        drivingConfig.idleMode(SparkBaseConfig.IdleMode.kBrake).smartCurrentLimit(50);
        drivingConfig
                .encoder
                .positionConversionFactor(drivingFactor)
                .velocityConversionFactor(drivingFactor / 60.0);
        drivingConfig
                .closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(0, 0, 0)
//                .velocityFF(0.0)
                .outputRange(-1, 1);

        turningConfig.idleMode(SparkBaseConfig.IdleMode.kBrake).smartCurrentLimit(20);
        turningConfig
                .absoluteEncoder
                .inverted(false)
                .positionConversionFactor(turningFactor)
                .velocityConversionFactor(turningFactor / 60.0);
        turningConfig
                .closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .pid(0, 0, 0)
                .outputRange(-1, 1)
                .positionWrappingEnabled(true)
                .positionWrappingInputRange(0, turningFactor);
    }
}
