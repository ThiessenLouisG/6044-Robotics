// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    // Port numbers for driver and operator gamepads. These correspond with the numbers on the USB
    // tab of the DriverStation
    public static final int kDriverControllerPort = 0;
  }

  public static class DrivetrainConstants {
    // PWM ports/CAN IDs for motor controllers
    public static final int kLeftRearID = 3;
    public static final int kLeftFrontID = 2;
    public static final int kRightRearID = 0;
    public static final int kRightFrontID = 1;

    // Current limit for drivetrain motors
    public static final int kCurrentLimit = 60;

    public static final double kMaxSpeed = 3.0;
    public static final double kMaxAngularSpeed = 2 * Math.PI;

    public static final double kTrackWidth = 0.55245; // meters
    public static final double kWheelRadius = 0.0762; // meters
    public static final int kEncoderResolution = 4096;
  }

  public static class LauncherConstants {
    // PWM ports/CAN IDs for motor controllers
    public static final int kFeederID = 5;
    public static final int kLeftLauncherID = 6;
    public static final int kRightLauncherID = 7;
    // Current limit for launcher and feed wheels
    public static final int kLauncherCurrentLimit = 80;
    public static final int kFeedCurrentLimit = 80;

    // Speeds for wheels when intaking and launching. Intake speeds are negative to run the wheels
    // in reverse
    public static final double kLeftLauncherSetpoint = 6000;

    public static final double kRightLauncherSetpoint = 6000;
    public static final double kLaunchFeederSpeed = 1;
    public static final double kIntakeLeftLauncherSpeed = -2000;
    public static final double kIntakeRightLauncherSpeed = -2000;
    public static final double kIntakeFeederSpeed = -.5;

    public static final double kLauncherDelay = 1;

    public static final double kRightWheelRadius = 0.0381;
    public static final double kLeftWheelRadius = 0.0508;
    public static final int kEncoderResolution = 42;
  }

  public static class IntakeConstants {
    public static final int kIntakeID = 8;

    public static final double kIntakeSpeed = 1;
    public static final double kIntakePrepareLaunchSpeed = -.5;
    public static final double kIntakeLaunchSpeed = .75;

    public static final double kPrepareIntakeDelay = .2;
  }
}
