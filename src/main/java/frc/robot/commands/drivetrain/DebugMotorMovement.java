package frc.robot.commands.drivetrain;

import java.io.BufferedWriter;
import java.lang.module.ModuleDescriptor.Requires;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.stream.Collectors;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.commands.drivetrain.FunnyNumber.sdlog;
import static frc.robot.commands.drivetrain.FunnyNumber.funnynumber;

public class DebugMotorMovement extends CommandBase {
    public static double GEARBOX_RATIO = 48;
    public static double MAGIC_NUMBER = funnynumber("funny spool circumfrence (radians)", 1/GEARBOX_RATIO);
    public static double ENCODER_TICKS_TO_ROTATIONS = 1/2048;
    public static int PID_SLOT = 0;
    public WPI_TalonFX motorController;
    public String label;
    private DoubleSupplier setpointSupplier;
    private ElevatorFeedforward feedforward;
    private BufferedWriter writer;
    private double prev_velocity;
    private double prev_timestamp;
    public DebugMotorMovement(String label, WPI_TalonFX motorController, DoubleSupplier setpointSupplier, ElevatorFeedforward feedforward) {
        this.motorController = motorController;
        this.motorController.configNeutralDeadband(0.0001);
        this.label = label;
        this.setpointSupplier = setpointSupplier;
        this.feedforward = feedforward;

        this.prev_velocity = 0;
        this.prev_timestamp = Timer.getFPGATimestamp();

        this.motorController.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, PID_SLOT, 0);

        try {
            this.writer = Files.newBufferedWriter(Paths.get(label + "-log.csv"));
            this.writer.write("Position rot, Velocity rpm, Acceleration rpmps");
        } catch (Exception e) {}
    }
    private void log(double... items) {
        String line = List.of(items).stream().map(String::valueOf).collect(Collectors.joining(", "));
        if (this.writer != null) {
            try {
                this.writer.write(line + "\n");
            } catch (Exception e) {}
        }
        System.out.println("MOTOR DEBUG STAT - " + this.label + " " + line.toString());
    }
    public void execute() {
        this.motorController.setVoltage(3);

        // this.motorController.set(funnynumber("motor set", 0.1));


        // SmartDashboard.putNumber("scre", this.feedforward.calculate(this.setpointSupplier.getAsDouble()));
        // this.motorController.setVoltage(this.feedforward.calculate(this.setpointSupplier.getAsDouble(), 0));
        this.motorController.setVoltage(
            sdlog("feedforward output", this.feedforward.calculate(
                this.setpointSupplier.getAsDouble(),
                0
            ))
        );
        SmartDashboard.putNumber("actual output voltage", motorController.get());




                // this.motorController.getSelectedSensorPosition()*ENCODER_TICKS_TO_ROTATIONS / 2 / Math.PI,
        log(
            sdlog("position rot", this.motorController.getSelectedSensorPosition(PID_SLOT) * ENCODER_TICKS_TO_ROTATIONS * MAGIC_NUMBER),
            sdlog("velocity rpm", this.motorController.getSelectedSensorVelocity(PID_SLOT) * ENCODER_TICKS_TO_ROTATIONS * MAGIC_NUMBER * 10),
            sdlog("accelera rpmps", (this.motorController.getSelectedSensorVelocity(PID_SLOT) - prev_velocity)/(Timer.getFPGATimestamp()-prev_timestamp))
        );
        prev_velocity = this.motorController.getSelectedSensorVelocity(PID_SLOT);
        prev_timestamp = Timer.getFPGATimestamp();
    }
    public boolean isFinished() {
        return false;
    }
    public void end(boolean inturrupted) {
        if (inturrupted) System.out.println("based");
        try {
            this.writer.close();
        } catch (Exception e) {}
    }
}
