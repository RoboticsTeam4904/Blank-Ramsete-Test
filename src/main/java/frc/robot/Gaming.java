package frc.robot;

import java.lang.module.ModuleDescriptor.Requires;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class Gaming extends CommandBase {
    private final DriveSubsystem chassis;
    private final WPI_TalonFX funnyMotor;
    public Gaming(DriveSubsystem chassis) {
        this.chassis = chassis;
        this.funnyMotor = new WPI_TalonFX(1);
        addRequirements(this.chassis);
    }

    @Override
    public void initialize() {
        super.initialize();
        chassis.tankDriveVolts(6.0, 6.0);
        funnyMotor.setVoltage(6);
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
