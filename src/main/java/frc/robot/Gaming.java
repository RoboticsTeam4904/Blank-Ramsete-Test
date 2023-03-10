package frc.robot;

import java.lang.module.ModuleDescriptor.Requires;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class Gaming extends CommandBase {
    private final DriveSubsystem chassis;
    public Gaming(DriveSubsystem chassis) {
        this.chassis = chassis;
        addRequirements(this.chassis);
    }

    @Override
    public void initialize() {
        super.initialize();
        chassis.tankDriveVolts(6.0, 6.0);
    }

    @Override
    public void execute() {
        System.out.println("I am inside your home.");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
