// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.drivetrain.Balance;
import frc.robot.subsystems.DriveSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private int ticker = 0;
  private boolean once = true;
  private RobotContainer m_robotContainer;

  private WPI_TalonFX funnyMotor1;
  private WPI_TalonFX funnyMotor2;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    SmartDashboard.putNumber("left voltage", RobotContainer.Component.leftATalonFX.getMotorOutputVoltage());
    SmartDashboard.putNumber("right voltage", RobotContainer.Component.rightATalonFX.getMotorOutputVoltage());
    SmartDashboard.putString("Pose", m_robotContainer.m_robotDrive.getPose().toString());
    SmartDashboard.putNumber("pose X", m_robotContainer.m_robotDrive.getPose().getX());
    if (ticker % 10 == 0) {
      // System.out.println(m_robotContainer.m_robotDrive.getPose().getX());
    }
    ticker++;
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    RobotContainer.Component.leftATalonFX.setNeutralMode(NeutralMode.Brake);
    RobotContainer.Component.leftBTalonFX.setNeutralMode(NeutralMode.Brake);
    RobotContainer.Component.rightATalonFX.setNeutralMode(NeutralMode.Brake);
    RobotContainer.Component.rightBTalonFX.setNeutralMode(NeutralMode.Brake);

    // m_autonomousCommand = new Balance(m_robotContainer.m_robotDrive.m_gyro,
    // new SimpleMotorFeedforward(
    // DriveConstants.ksVolts,
    // DriveConstants.kvVoltSecondsPerMeter,
    // DriveConstants.kaVoltSecondsSquaredPerMeter),
    // new PIDController(DriveConstants.kPDriveVel, 0.1, 0.00001),
    // new PIDController(DriveConstants.kPDriveVel, 0.1, 0.00001),
    // m_robotContainer.m_robotDrive::getWheelSpeeds,
    // // RamseteCommand passes volts to the callback
    // m_robotContainer.m_robotDrive::tankDriveVolts, 0.5, 0.1,
    // m_robotContainer.m_robotDrive);
    
    // String trajectoryJSON = "output/haha.wpilib.json";
    // String trajectoryJSON = "pathplanner/generatedJSON/compact_backup_test.wpilib.json";
    // String trajectoryJSON = "pathplanner/generatedJSON/long_turnless.wpilib.json";
    // String trajectoryJSON = "pathplanner/generatedJSON/big_wide_turns.wpilib.json";
    String trajectoryJSON = "pathplanner/generatedJSON/no backup.wpilib.json";

    Trajectory trajectory = new Trajectory();
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      System.out.println("v\nv\nv\nv\ntrajectory total time" + String.valueOf(trajectory.getTotalTimeSeconds()));
    } catch (IOException ex) {
      System.out.println("SHEEEEEESH");
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }

    m_autonomousCommand = m_robotContainer.getAutonomousCommand(trajectory);
    // m_autonomousCommand = new Gaming(m_robotContainer.m_robotDrive);
    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null && once) {
      m_autonomousCommand.schedule();
      once = false;
    } else {
      once = true;
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    RobotContainer.Component.leftATalonFX.setNeutralMode(NeutralMode.Coast);
    RobotContainer.Component.leftBTalonFX.setNeutralMode(NeutralMode.Coast);
    RobotContainer.Component.rightATalonFX.setNeutralMode(NeutralMode.Coast);
    RobotContainer.Component.rightBTalonFX.setNeutralMode(NeutralMode.Coast);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    funnyMotor1 = new WPI_TalonFX(15);
    funnyMotor1.setInverted(InvertType.OpposeMaster);
    funnyMotor2 = new WPI_TalonFX(9);
    funnyMotor1.follow(funnyMotor2);
    funnyMotor2.configNeutralDeadband(0.0001);
    funnyMotor1.configNeutralDeadband(0.0001);
    funnyMotor2.set(0.02);
    
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }
}
