package frc.robot.commands.drivetrain;

import java.io.BufferedWriter;
import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.stream.Collectors;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DebugMotorMovement extends CommandBase {
    public static double MAGIC_NUMBER = 0.0798;
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
        System.out.println("MOTOR DEBUG STAT - " + this.label + " " + line);
    }
    public void execute() {
        this.motorController.setVoltage(
            this.feedforward.calculate(
                this.setpointSupplier.getAsDouble(),
                0
            )/RobotController.getBatteryVoltage()
        );
                // this.motorController.getSelectedSensorPosition()*ENCODER_TICKS_TO_ROTATIONS / 2 / Math.PI,
        log(
            this.motorController.getSelectedSensorPosition(PID_SLOT) * ENCODER_TICKS_TO_ROTATIONS * MAGIC_NUMBER,
            this.motorController.getSelectedSensorVelocity(PID_SLOT) * ENCODER_TICKS_TO_ROTATIONS * MAGIC_NUMBER * 10,
            (this.motorController.getSelectedSensorVelocity(PID_SLOT) - prev_velocity)/(Timer.getFPGATimestamp()-prev_timestamp)
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
