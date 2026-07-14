// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autonomous.SuperSecretMissileTech;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.DriveToPoint;
import frc.robot.commands.RotateToAngle;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Vision;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

    final CommandPS5Controller primary_controller = new CommandPS5Controller(0);
    final CommandXboxController secondary_controller = new CommandXboxController(1);
    private final SwerveSubsystem swerve = new SwerveSubsystem();
    private final SuperSecretMissileTech superSecretMissileTech = new SuperSecretMissileTech(swerve);
    private final Vision vision = new Vision(swerve);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer()
    {
        configureBindings();
        DriverStation.silenceJoystickConnectionWarning(true);
    }


    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
     * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
     * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
     * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
     */

    private void configureBindings()
    {
        swerve.setDefaultCommand(

                new DriveCommand(
                        swerve,
                        () -> -1 * primary_controller.getLeftX(),
                        primary_controller::getLeftY,
                        () -> -1 * primary_controller.getRightX()
                )
        );

        primary_controller.options().onTrue(new InstantCommand(swerve::zeroGyro));
        primary_controller.pov(180).onTrue(new InstantCommand(() -> swerve.resetOdometry(FieldConstants.START_POINT)));
        primary_controller.cross().whileTrue(
            new RotateToAngle(swerve, Rotation2d.kZero)
        );

//        secondary_controller.rightStick().onTrue(new RefreshPreferences(swerve));

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */

    public Command getAutonomousCommand()
    {
        return superSecretMissileTech.getSelected();
    }

}
