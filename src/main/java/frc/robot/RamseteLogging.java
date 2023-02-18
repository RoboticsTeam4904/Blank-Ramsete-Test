package frc.robot;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.function.BiConsumer;
import java.util.function.Supplier;
import java.util.stream.Collectors;

import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class RamseteLogging extends RamseteCommand{
    double elapsed = 0; 
    ScheduledExecutorService logger = Executors.newScheduledThreadPool(1);
    ArrayList<ArrayList> actualdata = new ArrayList<ArrayList>();

    public RamseteLogging(Trajectory trajectory,
    Supplier<Pose2d> pose,
    RamseteController controller,
    SimpleMotorFeedforward feedforward,
    DifferentialDriveKinematics kinematics,
    Supplier<DifferentialDriveWheelSpeeds> wheelSpeeds,
    PIDController leftController,
    PIDController rightController,
    BiConsumer<Double, Double> outputVolts,
    Subsystem... requirements) {
        super(trajectory, pose, controller, feedforward, kinematics, wheelSpeeds, leftController, rightController, outputVolts, requirements);
    }
    public void initialize() {
        super.initialize();
        logger.scheduleAtFixedRate(() -> {
            DifferentialDriveKinematics kinematics = Constants.DriveConstants.kDriveKinematics;
            var wheel_speeds = RobotContainer.Component.m_robotDrive.getWheelSpeeds();
            var chassis_speeds = kinematics.toChassisSpeeds(wheel_speeds);
            var actual_velocity = chassis_speeds.vxMetersPerSecond;
            var actual_curvature = chassis_speeds.omegaRadiansPerSecond/actual_velocity;
            var actual_x = RobotContainer.Component.m_robotDrive.getPose().getX();
            var actual_y = RobotContainer.Component.m_robotDrive.getPose().getY();
            actualdata.add(new ArrayList<Double>(Arrays.asList(elapsed, actual_velocity, actual_curvature, actual_x, actual_y)));
            elapsed += 0.02;
        }, 0, 20, TimeUnit.MILLISECONDS);
    }
    public void execute() {
        super.execute();
    }
    public void end(boolean interrupted) {
        super.end(interrupted);
        try {
            FileWriter myWriter = new FileWriter("/home/lvuser/actualdata.json");
            myWriter.write(actualdata.stream().map(Object::toString).collect(Collectors.joining(", ", "[", "]")));
            myWriter.close();
        } catch (IOException e) {
            System.out.println("Failed to write actual data to file," + e);
        }
        logger.shutdownNow();
    }
    public boolean isFinished() {
        System.out.println("Finished: " + super.isFinished());
        return super.isFinished();
    }   
}
