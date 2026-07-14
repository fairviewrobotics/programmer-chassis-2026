package frc.robot.subsystems;

import com.studica.frc.AHRS;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.SwerveConstants;
import org.littletonrobotics.junction.Logger;

public class SwerveSubsystem extends SubsystemBase {

    private final SwerveModule frontLeft = new SwerveModule(SwerveConstants.FRONT_LEFT_DRIVING_CAN_ID, SwerveConstants.FRONT_LEFT_TURNING_CAN_ID, SwerveConstants.FRONT_LEFT_CHASSIS_ANGULAR_OFFSET);
    private final SwerveModule frontRight = new SwerveModule(SwerveConstants.FRONT_RIGHT_DRIVING_CAN_ID, SwerveConstants.FRONT_RIGHT_TURNING_CAN_ID, SwerveConstants.FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET);
    private final SwerveModule backLeft = new SwerveModule(SwerveConstants.REAR_LEFT_DRIVING_CAN_ID, SwerveConstants.REAR_LEFT_TURNING_CAN_ID, SwerveConstants.REAR_LEFT_CHASSIS_ANGULAR_OFFSET);
    private final SwerveModule backRight = new SwerveModule(SwerveConstants.REAR_RIGHT_DRIVING_CAN_ID, SwerveConstants.REAR_RIGHT_TURNING_CAN_ID, SwerveConstants.REAR_RIGHT_CHASSIS_ANGULAR_OFFSET);

    private final AHRS gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);
    private double simYaw = 0.0;

    private final SwerveDrivePoseEstimator poseEstimator;

    public SwerveSubsystem() {
        initializePreferences();

        this.poseEstimator = new SwerveDrivePoseEstimator(
                SwerveConstants.DRIVE_KINEMATICS,
                getRotation(),
                new SwerveModulePosition[] {
                        frontLeft.getPosition(),
                        frontRight.getPosition(),
                        backLeft.getPosition(),
                        backRight.getPosition()
                },
                Pose2d.kZero,
                VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
                VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30))
        );
    }

    public Rotation2d getRotation() {
        if (edu.wpi.first.wpilibj.RobotBase.isSimulation()) {
            return new Rotation2d(simYaw);
        }
        return gyro.getRotation2d();
    }

    public void drive(double xVel, double yVel, double omega) {
        var swerveModuleStates =
                SwerveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
                        ChassisSpeeds.discretize(
                                ChassisSpeeds.fromFieldRelativeSpeeds(
                                        xVel, yVel, omega, getRotation()),
                                0.02
                        )
                );

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.MAX_SPEED_METERS_PER_SECOND);
        frontLeft.setDesiredState(swerveModuleStates[0]);
        frontRight.setDesiredState(swerveModuleStates[1]);
        backLeft.setDesiredState(swerveModuleStates[2]);
        backRight.setDesiredState(swerveModuleStates[3]);
    }

    private void initializePreferences() {
        Preferences.initDouble("Swerve/DriveP", 0);
        Preferences.initDouble("Swerve/DriveD", 0);
        Preferences.initDouble("Swerve/DriveKV", 0);
        Preferences.initDouble("Swerve/TurningP", 0);
        Preferences.initDouble("Swerve/TurningD", 0);
    }

    public void addVisionMeasurement(Pose2d pose, double timestamp, Matrix<N3,N1> stdDevs) {
        if (poseEstimator != null) {
            poseEstimator.addVisionMeasurement(pose, timestamp, stdDevs);
        }
    }

    public void resetOdometry(Pose2d pose) {
        if (poseEstimator != null) {
            if (edu.wpi.first.wpilibj.RobotBase.isSimulation()) {
                simYaw = pose.getRotation().getRadians();
            }
            poseEstimator.resetPose(pose);
        }
    }

    public void zeroGyro() {
        gyro.reset();
        if (edu.wpi.first.wpilibj.RobotBase.isSimulation()) {
            simYaw = 0.0;
        }
    }

    public Pose2d getPose() {
        return poseEstimator != null ? poseEstimator.getEstimatedPosition() : Pose2d.kZero;
    }

    public ChassisSpeeds getFieldVelocity() {
        ChassisSpeeds robotRelativeSpeeds = SwerveConstants.DRIVE_KINEMATICS.toChassisSpeeds(
                frontLeft.getState(),
                frontRight.getState(),
                backLeft.getState(),
                backRight.getState()
        );
        return ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeSpeeds, getRotation());
    }

    @Override
    public void periodic() {
        if (poseEstimator == null) {
            return;
        }

        if (edu.wpi.first.wpilibj.RobotBase.isSimulation()) {
            frontLeft.updateSim(0.02);
            frontRight.updateSim(0.02);
            backLeft.updateSim(0.02);
            backRight.updateSim(0.02);

            ChassisSpeeds chassisSpeeds = SwerveConstants.DRIVE_KINEMATICS.toChassisSpeeds(
                    frontLeft.getState(),
                    frontRight.getState(),
                    backLeft.getState(),
                    backRight.getState()
            );
            simYaw += chassisSpeeds.omegaRadiansPerSecond * 0.02;
        }

        poseEstimator.update(
                getRotation(),
                new SwerveModulePosition[] {
                        frontLeft.getPosition(),
                        frontRight.getPosition(),
                        backLeft.getPosition(),
                        backRight.getPosition()
                }
        );

        Logger.recordOutput("Swerve/Pose", getPose());
        Logger.recordOutput("Swerve/ModuleStates",
                frontLeft.getState(),
                frontRight.getState(),
                backLeft.getState(),
                backRight.getState());
    }
}