package frc.robot.commands.drivetrain;

import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.function.BiConsumer;
import java.util.function.Supplier;

import com.kauailabs.navx.AHRSProtocol.AHRSUpdateBase;
import com.kauailabs.navx.frc.AHRS;
import com.kauailabs.navx.frc.ITimestampedDataSubscriber;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.BooleanArrayEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class Balance extends CommandBase {
    public static ArrayList<Pair<Long, Float>> debugObject = new ArrayList<Pair<Long, Float>>();
    private static float RAMP_START_ANGLE = 10.0f;
    private static float RAMP_BALANCE_TOLERANCE = 4.0f;

    private final SimpleMotorFeedforward feedforward;
    private final PIDController closedLeft;
    private final PIDController closedRight;
    private final TrapezoidProfile profile;
    private final AHRS gyro;

    private final Supplier<DifferentialDriveWheelSpeeds> wheelSpeeds;
    private final BiConsumer<Double, Double> outputVolts;

    private double initial_timestamp;
    private boolean onRamp = false;
    private boolean balanced = false;

    private boolean logging = false;
    public Balance(AHRS gyro, SimpleMotorFeedforward feedforward, PIDController left, PIDController right, Supplier<DifferentialDriveWheelSpeeds> wheelSpeeds, BiConsumer<Double, Double> outputVolts, double maxAccel, double maxVelocity, Subsystem... requirements) {
        addRequirements(requirements);
        this.profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(maxAccel, 0), new TrapezoidProfile.State(0, 0), new TrapezoidProfile.State(maxVelocity, maxAccel));

        this.feedforward = feedforward;
        this.closedLeft = left;
        this.closedRight = right;
        this.gyro = gyro;

        this.wheelSpeeds = wheelSpeeds;
        this.outputVolts = outputVolts;
    }

    @Override
    public void initialize() {
        initial_timestamp = Timer.getFPGATimestamp();
        if (logging) {
            gyro.registerCallback(new ITimestampedDataSubscriber() {
                @Override
                public void timestampedDataReceived(long system_timestamp, long sensor_timestamp, AHRSUpdateBase sensor_data, Object object) {
                    // System.out.println("I am living in your walls");
                    Balance.debugObject.add(new Pair<Long, Float>(system_timestamp, sensor_data.pitch));
                }
            }, new Object());
        }
    }

    @Override
    public void execute() {
        double elapsed_time = (Timer.getFPGATimestamp() - initial_timestamp);
        double setpoint = profile.calculate(elapsed_time).position;
        DifferentialDriveWheelSpeeds currentWheelspeeds = wheelSpeeds.get();

        double leftOutput = closedLeft.calculate(currentWheelspeeds.leftMetersPerSecond, setpoint)
                            + feedforward.calculate(setpoint);
        double rightOutput = closedRight.calculate(currentWheelspeeds.rightMetersPerSecond, setpoint)
                            + feedforward.calculate(setpoint);
    
        outputVolts.accept(leftOutput, rightOutput);
        
        if (logging) {
            // SmartDashboard.putBoolean("on ramp", onRamp);
            // SmartDashboard.putNumber("elapsed", elapsed_time);
            // SmartDashboard.putBoolean("balanced", balanced);
            // SmartDashboard.putNumber("left wheel speed", wheelSpeeds.get().leftMetersPerSecond);
            // SmartDashboard.putNumber("right wheel speed", wheelSpeeds.get().rightMetersPerSecond);
           
            // SmartDashboard.putNumber("left difference speed", wheelSpeeds.get().leftMetersPerSecond - setpoint);
            // SmartDashboard.putNumber("right difference speed", wheelSpeeds.get().rightMetersPerSecond - setpoint);


            System.out.println(elapsed_time);
            System.out.println(onRamp);
            System.out.println(balanced);


            System.out.println("left difference speed " +  (wheelSpeeds.get().leftMetersPerSecond - setpoint));
            System.out.println("right difference speed " +  (wheelSpeeds.get().rightMetersPerSecond - setpoint));
        }


        if (gyro.getPitch() > Math.abs(RAMP_START_ANGLE) && !onRamp) {
            onRamp = true;
        }
    }

    @Override
    public boolean isFinished() {
        if (onRamp && Math.abs(gyro.getPitch()) < RAMP_BALANCE_TOLERANCE) {
            balanced = true;
            // return true;
        } //else {
        //     return false;
        // }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        if (logging) {
            try {
                FileWriter file = new FileWriter("gyrolog.json");
                file.write(Balance.debugObject.toString());
                file.close();
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
        outputVolts.accept(0.0, 0.0);
    }
}
