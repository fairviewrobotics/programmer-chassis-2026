package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveToPoint extends Command {

    SwerveSubsystem swerve;

    public DriveToPoint() {
        this.swerve = swerve;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(0, 0, 0);
    }

}
