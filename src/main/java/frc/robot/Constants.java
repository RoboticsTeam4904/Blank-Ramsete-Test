// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 * 
 * TO-DO: update these constants to match your robot
 */
public final class Constants {
  public static final class DriveConstants {
    public static final int kLeftMotor1Port = 3;
    public static final int kLeftMotor2Port = 5;
    public static final int kRightMotor1Port = 2;
    public static final int kRightMotor2Port = 4;

    public static final int[] kLeftEncoderPorts = new int[] {3, 5};
    public static final int[] kRightEncoderPorts = new int[] {2, 4};
    public static final boolean kLeftEncoderReversed = true;
    public static final boolean kRightEncoderReversed = false;

    // public static final double kTrackwidthMeters = 0.50367; //blinky's track width
    public static final double kTrackwidthMeters = 0.59; //blinky's track width
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);

    public static final int kEncoderCPR = 2048;
    public static final double kWheelDiameterMeters = 0.1016;
    public static final double gearRatio = 69/5;
    //using blinky gear ratio
    public static final double kEncoderDistancePerPulse = kWheelDiameterMeters*Math.PI / kEncoderCPR / gearRatio;
        // Assumes the encoders are directly mounted on the wheel shafts
        // (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR / 17;

    public static final double ksVolts = 0.0081094; //old is 0.27395 
    public static final double kvVoltSecondsPerMeter = 4.7873; //was consistent between the two
    public static final double kaVoltSecondsSquaredPerMeter = 0.13655; //old is 0.25289
    public static final double kPDriveVel = 0.1771; //old is 0.049489
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 4.0; // max like 6mps
    public static final double kMaxAccelerationMetersPerSecondSquared = 4.0;

    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.1;
  }
}
