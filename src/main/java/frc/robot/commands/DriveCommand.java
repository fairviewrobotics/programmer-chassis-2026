package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.DoubleSupplier;

public class DriveCommand extends Command {
    private final SwerveSubsystem swerve;
    private final DoubleSupplier xVel, yVel, omega;

    public DriveCommand(
            SwerveSubsystem swerve,
            DoubleSupplier xVel,
            DoubleSupplier yVel,
            DoubleSupplier omega
    ) {
        this.swerve = swerve;
        this.xVel = xVel;
        this.yVel = yVel;
        this.omega = omega;

        addRequirements(swerve);
    }

    @Override
    public void execute() {
        double xSpeed = xVel.getAsDouble();
        double ySpeed = yVel.getAsDouble();
        double turningSpeed = omega.getAsDouble();

        xSpeed = MathUtil.applyDeadband(xSpeed, 0.1);
        ySpeed = MathUtil.applyDeadband(ySpeed, 0.1);
        turningSpeed = MathUtil.applyDeadband(turningSpeed, 0.1);

        double xVel = xSpeed * SwerveConstants.MAX_DRIVE_VELOCITY_MPS;
        double yVel = ySpeed * SwerveConstants.MAX_DRIVE_VELOCITY_MPS;
        double turningVel = turningSpeed * (Math.PI * 2);

        swerve.drive(xVel, yVel, turningVel);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(0, 0, 0);
    }
}
