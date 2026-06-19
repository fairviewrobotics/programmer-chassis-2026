/* Black Knights Robotics (C) 2025 */
package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class SwerveModule extends SubsystemBase {
    private final SparkFlex drivingSpark;
    private final SparkMax turningSpark;

    private final int drivingCanId;

    private final RelativeEncoder drivingEncoder;
    private final AbsoluteEncoder turningEncoder;

    private final SparkClosedLoopController drivingClosedLoopController;
    private final SparkClosedLoopController turningClosedLoopController;

    private final double chassisAngularOffset;
    private SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d());

    private SimpleMotorFeedforward feedforward =
            new SimpleMotorFeedforward(0, 0, 0, 0);

    public SwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
        drivingSpark = new SparkFlex(drivingCANId, MotorType.kBrushless);
        turningSpark = new SparkMax(turningCANId, MotorType.kBrushless);
        this.drivingCanId = drivingCANId;

        drivingEncoder = drivingSpark.getEncoder();
        turningEncoder = turningSpark.getAbsoluteEncoder();

        drivingClosedLoopController = drivingSpark.getClosedLoopController();
        turningClosedLoopController = turningSpark.getClosedLoopController();

        drivingSpark.configure(
                SwerveModuleConfig.drivingConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        turningSpark.configure(
                SwerveModuleConfig.turningConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        this.chassisAngularOffset = chassisAngularOffset;
        desiredState.angle = new Rotation2d(turningEncoder.getPosition());
        drivingEncoder.setPosition(0);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
                drivingEncoder.getVelocity(),
                new Rotation2d(turningEncoder.getPosition() - chassisAngularOffset));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                drivingEncoder.getPosition(),
                new Rotation2d(turningEncoder.getPosition() - chassisAngularOffset));
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        SwerveModuleState correctedDesiredState = new SwerveModuleState();
        correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
        correctedDesiredState.angle =
                desiredState.angle.plus(Rotation2d.fromRadians(chassisAngularOffset));

        correctedDesiredState.optimize(new Rotation2d(turningEncoder.getPosition()));

        double ffOutput =
                feedforward.calculateWithVelocities(
                        drivingEncoder.getVelocity(), correctedDesiredState.speedMetersPerSecond);

        drivingClosedLoopController.setSetpoint(
                correctedDesiredState.speedMetersPerSecond,
                ControlType.kVelocity,
                ClosedLoopSlot.kSlot0,
                ffOutput);

        turningClosedLoopController.setSetpoint(
                correctedDesiredState.angle.getRadians(), ControlType.kPosition);

        this.desiredState = desiredState;
    }

    public void reconfigure(SparkFlexConfig drivingConfig, SparkMaxConfig turningConfig) {
        drivingSpark.configure(
                drivingConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        turningSpark.configure(
                turningConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void resetEncoders() {
        drivingEncoder.setPosition(0);
    }

    public void setTurningVoltage(double voltage) {
        this.turningSpark.setVoltage(voltage);
    }

    public void setDrivingVoltage(double voltage) {
        this.drivingSpark.setVoltage(voltage);
    }
}
