// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj.XboxController.Button;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.RamseteLogging;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;


import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */



public class RobotContainer {
  
    public static class Component{
        public static WPI_TalonFX leftATalonFX;
        public static WPI_TalonFX leftBTalonFX;
        public static WPI_TalonFX rightATalonFX;
        public static WPI_TalonFX rightBTalonFX;
        public static DriveSubsystem m_robotDrive;
}

// The robot's subsystems
    //motors
     


  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    Component.leftATalonFX = new WPI_TalonFX(Constants.DriveConstants.kLeftMotor1Port);
    Component.leftBTalonFX = new WPI_TalonFX(Constants.DriveConstants.kLeftMotor2Port);
    Component.leftATalonFX.setInverted(true);
    Component.leftBTalonFX.setInverted(true);
    Component.rightATalonFX = new WPI_TalonFX(Constants.DriveConstants.kRightMotor1Port);
    Component.rightBTalonFX = new WPI_TalonFX(Constants.DriveConstants.kRightMotor2Port);
    System.out.println("motorcontrollers created");
    Component.m_robotDrive = new DriveSubsystem();

    // Configure default commands
    // Set the default drive command to split-stick arcade drive
    Component.m_robotDrive.setDefaultCommand(
        // A split-stick arcade command, with forward/backward controlled by the left
        // hand, and turning controlled by the right.
        new RunCommand(
            () ->
                Component.m_robotDrive.arcadeDrive(
                    -m_driverController.getLeftY(), -m_driverController.getRightX()),
            Component.m_robotDrive));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    // Drive at half speed when the right bumper is held
    new JoystickButton(m_driverController, Button.kRightBumper.value)
        .onTrue(new InstantCommand(() -> Component.m_robotDrive.setMaxOutput(0.5)))
        .onFalse(new InstantCommand(() -> Component.m_robotDrive.setMaxOutput(1)));
    new JoystickButton(m_driverController, Button.kA.value)
        .onTrue(getAutonomousCommand()); 
    }
    


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                DriveConstants.ksVolts,
                DriveConstants.kvVoltSecondsPerMeter,
                DriveConstants.kaVoltSecondsSquaredPerMeter),
            DriveConstants.kDriveKinematics,
            10);

    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)),
            // Pass config
            config);

    RamseteLogging ramseteCommand =
        new RamseteLogging(
            exampleTrajectory,
            Component.m_robotDrive::getPose,
            new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
            new SimpleMotorFeedforward(
                DriveConstants.ksVolts,
                DriveConstants.kvVoltSecondsPerMeter,
                DriveConstants.kaVoltSecondsSquaredPerMeter),
            DriveConstants.kDriveKinematics,
            Component.m_robotDrive::getWheelSpeeds,
            new PIDController(DriveConstants.kPDriveVel, 0, 0),
            new PIDController(DriveConstants.kPDriveVel, 0, 0),
            // RamseteCommand passes volts to the callback
            Component.m_robotDrive::tankDriveVolts,
            Component.m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.

    Component.m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());
    
    
    ArrayList<ArrayList> TrajectoryData = new ArrayList<ArrayList>(); 
    for (double elapsed=0; elapsed<exampleTrajectory.getTotalTimeSeconds(); elapsed += 0.02) {
      var mmm = exampleTrajectory.sample(elapsed);
      SmartDashboard.putNumber("Intended Trajectory elapsed_time", mmm.timeSeconds);
      SmartDashboard.putNumber("Intended Trajectory velocity", mmm.velocityMetersPerSecond);
      SmartDashboard.putNumber("Intended Trajectory poseX", mmm.poseMeters.getX());
      SmartDashboard.putNumber("Intended Trajectory poseY", mmm.poseMeters.getY());
      SmartDashboard.putNumber("Intended Trajectory curvature", mmm.curvatureRadPerMeter);
      TrajectoryData.add(new ArrayList<Double>(Arrays.asList(mmm.timeSeconds, mmm.velocityMetersPerSecond, mmm.curvatureRadPerMeter, mmm.poseMeters.getX(), mmm.poseMeters.getY())));
    }
    //write trajectory data to a json file
    try {
      FileWriter myWriter = new FileWriter("/home/lvuser/trajectoryData.json");
      myWriter.write(TrajectoryData.stream().map(Object::toString).collect(Collectors.joining(", ", "[", "]")));
      myWriter.close();
    } catch (IOException e) {
        System.out.println("Failed to write ideal trajectory data," + e);
  }

// Run path following command, then stop at the end.
return ramseteCommand.andThen(() -> Component.m_robotDrive.tankDriveVolts(0, 0));
  

}
}
