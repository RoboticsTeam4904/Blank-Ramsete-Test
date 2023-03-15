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
import frc.robot.commands.drivetrain.RamseteCommandDebug;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;
import frc.robot.Gaming;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */

public class RobotContainer {

        public static class Component {
                public static WPI_TalonFX leftATalonFX;
                public static WPI_TalonFX leftBTalonFX;
                public static WPI_TalonFX rightATalonFX;
                public static WPI_TalonFX rightBTalonFX;
                public static WPI_TalonFX testTalon;
        }

        // The robot's subsystems
        // motors

        public final DriveSubsystem m_robotDrive;
        public final XboxController m_driverController;

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {

                // The driver's controller
                m_driverController = new XboxController(OIConstants.kDriverControllerPort);

                // Configure the button bindings
                Component.leftATalonFX = new WPI_TalonFX(Constants.DriveConstants.kLeftMotor1Port);
                Component.leftBTalonFX = new WPI_TalonFX(Constants.DriveConstants.kLeftMotor2Port);

                Component.rightATalonFX = new WPI_TalonFX(Constants.DriveConstants.kRightMotor1Port);
                Component.rightBTalonFX = new WPI_TalonFX(Constants.DriveConstants.kRightMotor2Port);

		
		RobotContainer.Component.leftATalonFX.setNeutralMode(NeutralMode.Coast);
		RobotContainer.Component.leftBTalonFX.setNeutralMode(NeutralMode.Coast);
		RobotContainer.Component.rightATalonFX.setNeutralMode(NeutralMode.Coast);
		RobotContainer.Component.rightBTalonFX.setNeutralMode(NeutralMode.Coast);

                Component.testTalon = new WPI_TalonFX(1);
                // Component.leftATalonFX.setInverted(true);
                // Component.leftBTalonFX.setInverted(true);

                this.m_robotDrive = new DriveSubsystem();

                // Configure default commands
                // Set the default drive command to split-stick arcade drive
                //m_robotDrive.setDefaultCommand(
                //                new RunCommand(() -> System.out.println("I am inside your home"), m_robotDrive));
                // // A split-stick arcade command, with forward/backward controlled by the left
                // // hand, and turning controlled by the right.
                // new RunCommand(
                // () -> m_robotDrive.arcadeDrive(
                // -m_driverController.getLeftY(), -m_driverController.getRightX()),
                // m_robotDrive));
                configureButtonBindings();
        }

        /**
         * Use this method to define your button->command mappings. Buttons can be
         * created by
         * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
         * subclasses ({@link
         * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
         * passing it to a
         * {@link JoystickButton}.
         */
        private void configureButtonBindings() {
                // Drive at half speed when the right bumper is held
                new JoystickButton(m_driverController, Button.kRightBumper.value)
                                .onTrue(new InstantCommand(() -> m_robotDrive.setMaxOutput(0.5)))
                                .onFalse(new InstantCommand(() -> m_robotDrive.setMaxOutput(1)));
                new JoystickButton(m_driverController, Button.kA.value)
                                .onTrue(getAutonomousCommand());
        }
	
	public Command getAutonomousCommand(Trajectory trajectory) {
                var ramseteController = new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta);

		RamseteCommandDebug ramseteCommand = new RamseteCommandDebug(
		// RamseteCommand ramseteCommand = new RamseteCommand(
			trajectory,
			m_robotDrive::getPose,
                        ramseteController,
			new SimpleMotorFeedforward(
				DriveConstants.ksVolts,
				DriveConstants.kvVoltSecondsPerMeter,
				DriveConstants.kaVoltSecondsSquaredPerMeter),
			DriveConstants.kDriveKinematics,
			m_robotDrive::getWheelSpeeds,
			new PIDController(DriveConstants.kPDriveVel*6, 0, 0),
			new PIDController(DriveConstants.kPDriveVel*6, 0, 0),
			// RamseteCommand passes volts to the callback
			m_robotDrive::tankDriveVolts,
			m_robotDrive);
	
		// Reset odometry to the starting pose of the trajectory.
		Pose2d initialPose = trajectory.getInitialPose();
		m_robotDrive.resetOdometry(initialPose);
		SmartDashboard.putString("initial pose", initialPose.toString());
		// return new Gaming(m_robotDrive);
		// Run path following command, then stop at the end.
		// return Commands.run(() -> m_robotDrive.tankDriveVolts(1, 1), m_robotDrive);
		//return Commands.runOnce(() -> m_robotDrive.arcadeDrive(0.5, 0), m_robotDrive);
		//return Commands.runOnce(() -> Component.testTalon.setVoltage(6));
		return ramseteCommand
			.andThen(() -> m_robotDrive.tankDriveVolts(0, 0))
			.andThen((new CommandBase(){}).withTimeout(2))
			.andThen(Commands.runOnce(() -> {
				RobotContainer.Component.leftATalonFX.setNeutralMode(NeutralMode.Coast);
				RobotContainer.Component.leftBTalonFX.setNeutralMode(NeutralMode.Coast);
				RobotContainer.Component.rightATalonFX.setNeutralMode(NeutralMode.Coast);
				RobotContainer.Component.rightBTalonFX.setNeutralMode(NeutralMode.Coast);
		}));
	}

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
// Create a voltage constraint to ensure we don't accelerate too fast
        var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(
                        DriveConstants.ksVolts,
                        DriveConstants.kvVoltSecondsPerMeter,
                        DriveConstants.kaVoltSecondsSquaredPerMeter),
                DriveConstants.kDriveKinematics,
                10);

        // Create config for trajectory
        TrajectoryConfig config = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(DriveConstants.kDriveKinematics)
                // Apply the voltage constraint
                .addConstraint(autoVoltageConstraint);


        final double dist = 1;

        // An example trajectory to follow. All units in meters.
        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                // List.of(new Translation2d(0.33*dist, .15*dist), new Translation2d(0.66*dist, -.15*dist)),
                List.of(new Translation2d(1, -1), new Translation2d(2, -1)),

                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(2, 0, new Rotation2d(Math.PI/2)),
                // Pass config
                config);     
                

        // Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        //                 new Pose2d(0, 0, new Rotation2d(0)),
        //                 List.of(),
        //                 new Pose2d(1, -1, new Rotation2d(-Math.PI/2)),
        //                 config);
                
        // Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        //         // Start at the origin facing the +X direction
        //         new Pose2d(0, 0, new Rotation2d(0)),
        //         // Pass through these two interior waypoints, making an 's' curve path
        //         // List.of(new Translation2d(0.33*dist, .15*dist), new Translation2d(0.66*dist, -.15*dist)),
        //         List.of(new Translation2d(8, 0)),

        //         // End 3 meters straight ahead of where we started, facing forward
        //         new Pose2d(-1, 0, new Rotation2d(0)),
        //         // Pass config
        //         config);   
                
        System.out.println(this.m_robotDrive.toString());
	return this.getAutonomousCommand(exampleTrajectory);
    }
}
