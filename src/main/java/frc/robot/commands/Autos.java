// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.CANDrivetrain;
import frc.robot.subsystems.CANIntake;
import frc.robot.subsystems.CANLauncher;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static Command shootAuto(
      CANDrivetrain drivetrain, CANLauncher m_launcher, CANIntake m_intake) {
    /**
     * RunCommand is a helper class that creates a command from a single method, in this case we
     * pass it the arcadeDrive method to drive straight back at half power. We modify that command
     * with the .withTimeout(1) decorator to timeout after 1 second, and use the .andThen decorator
     * to stop the drivetrain after the first command times out
     */
    return new PrepareLaunch(m_launcher)
        .withTimeout(3)
        .andThen(
            new PrepareIntakeLaunch(m_intake).withTimeout(.1).andThen(new LaunchNote(m_intake)));

    // return new RunCommand(() -> drivetrain.arcadeDrive(1, .3))
    //     .withTimeout(2.5)
    //     .andThen(new RunCommand(() -> drivetrain.arcadeDrive(0, 0)));
  }

  public static Command driveShootAuto(
      CANDrivetrain drivetrain, CANLauncher m_launcher, CANIntake m_intake) {
    return Commands.sequence(
        Commands.print("Preparing Launcher"),
        new PrepareLaunch(m_launcher).withTimeout(2),
        Commands.print("Preparing intake for launch"),
        new PrepareIntakeLaunch(m_intake).withTimeout(.1),
        Commands.print("Launching"),
        new LaunchNote(m_intake).withTimeout(1),
        Commands.print("Drive 1"),
        new RunCommand(() -> drivetrain.arcadeDrive(.75, 0), drivetrain).withTimeout(.5),
        new RunCommand(() -> drivetrain.arcadeDrive(.6, -1), drivetrain).withTimeout(.75),
        Commands.print("Drive 2"),
        new RunCommand(() -> drivetrain.arcadeDrive(.75, 0), drivetrain).withTimeout(1),
        Commands.print("Drive 3"),
        new RunCommand(() -> drivetrain.arcadeDrive(.6, 1), drivetrain).withTimeout(.5),
        Commands.print("Drive 4"),
        new RunCommand(() -> drivetrain.arcadeDrive(.75, 0), drivetrain).withTimeout(1),
        Commands.print("Drive 5"),
        new InstantCommand(() -> drivetrain.arcadeDrive(0, 0), drivetrain),
        Commands.print("Driving done"));
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
