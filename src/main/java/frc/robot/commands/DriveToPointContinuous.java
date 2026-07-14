package frc.robot.commands;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.SwerveSubsystem;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

public class DriveToPointContinuous extends Command {

    private ProfiledPIDController thetaController;
    private SwerveSubsystem swerveSubsystem;
    private double speedMetersPerSecond;
    private double tolerance;
    private Pose2d targetLocation;
    private Supplier<Pose2d> targetLocationSupplier;

    private static final String AUTO_ROTATION_P_KEY = "DriveToPoint/AutoRotationP";

    public DriveToPointContinuous(
            SwerveSubsystem swerveSubsystem,
            Supplier<Pose2d> targetLocationSupplier,
            double speedMetersPerSecond,
            double tolerance) {
        this.swerveSubsystem = swerveSubsystem;
        this.targetLocationSupplier = targetLocationSupplier;
        this.speedMetersPerSecond = speedMetersPerSecond;
        this.tolerance = tolerance;

        Preferences.initDouble(AUTO_ROTATION_P_KEY, SwerveConstants.Turning_P);

        this.thetaController =
                new ProfiledPIDController(
                        Preferences.getDouble(AUTO_ROTATION_P_KEY, SwerveConstants.Turning_P),
                        0.0,
                        0.0,
                        new TrapezoidProfile.Constraints(
                                SwerveConstants.MAX_ANGULAR_SPEED,
                                SwerveConstants.MAX_ANGULAR_SPEED * 2.0),
                        0.02);

        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        addRequirements(swerveSubsystem);
    }

    public DriveToPointContinuous(
            SwerveSubsystem swerveSubsystem,
            Pose2d targetLocation,
            double speedMetersPerSecond,
            double tolerance) {
        this(swerveSubsystem, () -> targetLocation, speedMetersPerSecond, tolerance);
    }

    public DriveToPointContinuous(
            SwerveSubsystem swerveSubsystem,
            Pose2d targetLocation,
            double speedMetersPerSecond) {
        this(swerveSubsystem, targetLocation, speedMetersPerSecond, 0.25);
    }

    @Override
    public void initialize() {
        this.targetLocation = targetLocationSupplier.get();

        if (targetLocation == null) {
            return;
        }

        Pose2d currentPose = swerveSubsystem.getPose();

        thetaController.setP(Preferences.getDouble(AUTO_ROTATION_P_KEY, SwerveConstants.Turning_P));

        thetaController.reset(
                currentPose.getRotation().getRadians(),
                swerveSubsystem.getFieldVelocity().omegaRadiansPerSecond);
        thetaController.setTolerance(Units.degreesToRadians(5.0));
    }

    @Override
    public void execute() {
        if (targetLocation == null) {
            return;
        }

        Pose2d currentPose = swerveSubsystem.getPose();
        Logger.recordOutput("DriveToPointContinuous/current pose", currentPose);
        Logger.recordOutput("DriveToPointContinuous/target location", targetLocation);

        double currentDistance =
                currentPose.getTranslation().getDistance(targetLocation.getTranslation());

        Rotation2d angleToTarget = targetLocation.getTranslation().minus(currentPose.getTranslation()).getAngle();

        double vx = speedMetersPerSecond * angleToTarget.getCos();
        double vy = speedMetersPerSecond * angleToTarget.getSin();

        double thetaVelocity =
                thetaController.getSetpoint().velocity
                        + thetaController.calculate(
                        currentPose.getRotation().getRadians(),
                        targetLocation.getRotation().getRadians());

        double thetaErrorAbs =
                Math.abs(
                        currentPose.getRotation().minus(targetLocation.getRotation()).getRadians());
        if (thetaErrorAbs < thetaController.getPositionTolerance()) thetaVelocity = 0.0;

        swerveSubsystem.drive(vx, vy, thetaVelocity);
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            swerveSubsystem.drive(0.0, 0.0, 0.0);
        }
    }

    @Override
    public boolean isFinished() {
        if (targetLocation == null) {
            return true;
        }
        Pose2d currentPose = swerveSubsystem.getPose();
        double currentDistance = currentPose.getTranslation().getDistance(targetLocation.getTranslation());
        return currentDistance < tolerance;
    }
}
