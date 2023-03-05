package frc.robot.commands.drivetrain;

import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.function.BiConsumer;
import java.util.function.Supplier;

import com.apple.laf.resources.aqua;
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
    public static ArrayList<ArrayList> VelocityDebug = new ArrayList<ArrayList>();
    private static float RAMP_START_ANGLE = 9.0f;
    private static float RAMP_BALANCE_TOLERANCE = 4.0f;

    private final SimpleMotorFeedforward feedforward;
    private final PIDController closedLeft;
    private final PIDController closedRight;
    private final TrapezoidProfile profile;
    private final AHRS gyro;

    private final Supplier<DifferentialDriveWheelSpeeds> wheelSpeeds;
    private final BiConsumer<Double, Double> outputVolts;

    static double initial_timestamp_system = 0;
    static double prevtime = 0;
    static DifferentialDriveWheelSpeeds prevSpeed = new DifferentialDriveWheelSpeeds();
    static DifferentialDriveWheelSpeeds speed = new DifferentialDriveWheelSpeeds();




    private double initial_timestamp;
    private boolean onRamp = false;
    private boolean balanced = false;
    private double max_velocity;
    private boolean ACCELmode = false;
    private DifferentialDriveWheelSpeeds prevWheelSpeed;
    private static double s_setpoint;

    private boolean logging = true;
    public Balance(AHRS gyro, SimpleMotorFeedforward feedforward, PIDController left, PIDController right, Supplier<DifferentialDriveWheelSpeeds> wheelSpeeds, BiConsumer<Double, Double> outputVolts, double maxAccel, double maxVelocity, Subsystem... requirements) {
        addRequirements(requirements);
        this.profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(maxAccel, 0), new TrapezoidProfile.State(maxVelocity, maxAccel), new TrapezoidProfile.State(0, 0));
        this.s_setpoint = max_velocity;
        this.feedforward = feedforward;
        this.closedLeft = left;
        this.closedRight = right;
        this.gyro = gyro;
        this.max_velocity = maxVelocity;

        this.wheelSpeeds = wheelSpeeds;
        this.outputVolts = outputVolts;
    }

    @Override
    public void initialize() {
        initial_timestamp = Timer.getFPGATimestamp();
        ArrayList<String> titles = new ArrayList<String>();
        titles.add("System Time");
        titles.add("Right Wheel Speed");
        titles.add("Left Wheel Speed");
        titles.add("Setpoint");
        titles.add("Right Acceleration");
        titles.add("Left Acceleration");
        Balance.VelocityDebug.add(titles);
        if (logging) {
            gyro.registerCallback(new ITimestampedDataSubscriber() {
                @Override
                public void timestampedDataReceived(long system_timestamp, long sensor_timestamp, AHRSUpdateBase sensor_data, Object object) {
                    // System.out.println("I am living in your walls");
                    Balance.debugObject.add(new Pair<Long, Float>(system_timestamp, sensor_data.pitch));
                    if (initial_timestamp_system==0){
                        initial_timestamp_system = (double) system_timestamp; 
                    }
                    prevSpeed = speed;
                    speed = wheelSpeeds.get();
                    ArrayList<Double> velocities = new ArrayList<Double>();
                    velocities.add(initial_timestamp_system - (double) system_timestamp);
                    velocities.add(speed.rightMetersPerSecond);
                    velocities.add(speed.leftMetersPerSecond);
                    velocities.add(s_setpoint);
                    velocities.add((speed.leftMetersPerSecond-prevSpeed.leftMetersPerSecond)/(system_timestamp-prevtime));
                    velocities.add((speed.rightMetersPerSecond-prevSpeed.rightMetersPerSecond)/(system_timestamp-prevtime));
                    Balance.VelocityDebug.add(velocities);
                    prevtime = (double) system_timestamp;
                }
            }, new Object());
        }
    }

    @Override
    public void execute() {
        double elapsed_time = (Timer.getFPGATimestamp() - initial_timestamp);
        double setpoint = max_velocity;
        // double setpoint = profile.calculate(elapsed_time).position;
        DifferentialDriveWheelSpeeds currentWheelspeeds = wheelSpeeds.get();

        double leftOutput  = closedLeft .calculate(currentWheelspeeds.leftMetersPerSecond, setpoint);
                            // + feedforward.calculate(setpoint);
        double rightOutput = closedRight.calculate(currentWheelspeeds.rightMetersPerSecond, setpoint);
                            // + feedforward.calculate(setpoint);
    
        outputVolts.accept(leftOutput, rightOutput);
        if (logging) {
            SmartDashboard.putBoolean("on ramp", onRamp);
            SmartDashboard.putNumber(
                "elapsed", elapsed_time);
            SmartDashboard.putBoolean("balanced", balanced);
            SmartDashboard.putNumber("pitch", Math.abs(gyro.getPitch()));
            SmartDashboard.putNumber("left wheel speed", currentWheelspeeds.leftMetersPerSecond);
            SmartDashboard.putNumber("right wheel speed", currentWheelspeeds.rightMetersPerSecond);
            SmartDashboard.putNumber("setpoint", setpoint);
           
            SmartDashboard.putNumber("left difference speed", currentWheelspeeds.leftMetersPerSecond - setpoint);
            SmartDashboard.putNumber("right difference speed", currentWheelspeeds.rightMetersPerSecond - setpoint);

            if (ACCELmode) {
                SmartDashboard.putNumber("left acceleration", (currentWheelspeeds.leftMetersPerSecond-prevWheelSpeed.leftMetersPerSecond)/elapsed_time);
                SmartDashboard.putNumber("right acceleration", (currentWheelspeeds.rightMetersPerSecond-prevWheelSpeed.rightMetersPerSecond)/elapsed_time);

            }
            System.out.println("Setpoint" + setpoint);
            System.out.println();
            // System.out.println(elapsed_time);
            // System.out.println(onRamp);
            // System.out.println(balanced);


            System.out.println("left difference speed " +  (wheelSpeeds.get().leftMetersPerSecond - setpoint));
            System.out.println("right difference speed " +  (wheelSpeeds.get().rightMetersPerSecond - setpoint));
        }


        if (Math.abs(gyro.getPitch()) > RAMP_START_ANGLE && !onRamp) {
            onRamp = true;
        }
        prevWheelSpeed = currentWheelspeeds;
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
            try {
                FileWriter file = new FileWriter("speedLog.json");
                file.write(Balance.VelocityDebug.toString());
                file.close();
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
        outputVolts.accept(0.0, 0.0);
    }
}
