// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.IntakeConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANIntake;

// import frc.robot.subsystems.PWMLauncher;

/*This is an example of creating a command as a class. The base Command class provides a set of methods that your command
 * will override.
 */
public class LaunchNote extends Command {

  CANIntake m_intake;

  /** Creates a new LaunchNote. */
  public LaunchNote(CANIntake intake) {
    // save the launcher system internally
    // m_launcher = launcher;
    m_intake = intake;
    // indicate that this command requires the launcher system
    // addRequirements(m_launcher);
    addRequirements(m_intake);
  }

  // The initialize method is called when the command is initially scheduled.
  @Override
  public void initialize() {

    // Set the wheels to launching speed
    m_intake.setIntake(kIntakeLaunchSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // There is nothing we need this command to do on each iteration. You could remove this method
    // and the default blank method
    // of the base class will run.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Always return false so the command never ends on it's own. In this project we use the
    // scheduler to end the command when the button is released.
    return false;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stop the wheels when the command ends.
    m_intake.stop();
  }
}
