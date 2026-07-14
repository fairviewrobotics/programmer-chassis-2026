package frc.robot.autonomous.routines;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveToPoint;
import frc.robot.commands.DriveToPointContinuous;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.AllianceFlipUtil;

import java.util.Set;

public class ArcTestAuto extends SequentialCommandGroup {
    public ArcTestAuto(SwerveSubsystem swerveSubsystem) {
        setName("ARC TEST AUTO");
        addCommands(
                Commands.defer(() -> {
                    return new SequentialCommandGroup(
                            new InstantCommand(() -> {
                                Pose2d startPose = AllianceFlipUtil.apply(FieldConstants.BLUE_TRENCH_LEFT);
                                swerveSubsystem.resetOdometry(startPose);
                            }),
                            // Drive through transition point continuously to round the corner
                            new DriveToPointContinuous(swerveSubsystem, AllianceFlipUtil.apply(FieldConstants.BLUE_TRENCH_LEFT_TRANSITION_CONTINUOUS), 3.0, 0.4),
                            // Drive to final point and stop
                            new DriveToPoint(swerveSubsystem, AllianceFlipUtil.apply(FieldConstants.BLUE_TRENCH_LEFT_ARC_DROPOFF), 0.75)
                    );
                }, Set.of(swerveSubsystem))
        );
    }
}
