package frc.robot.commands;

import static frc.robot.Constants.IntakeConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANIntake;

public class Intake extends Command {
  CANIntake m_intake;

  /** Creates a new LaunchNote. */
  public Intake(CANIntake intake) {
    // save the launcher system internally
    m_intake = intake;

    // indicate that this command requires the launcher system
    addRequirements(m_intake);
  }

  @Override
  public void initialize() {
    m_intake.setIntake(kIntakeSpeed);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the wheels when the command ends.
    m_intake.stop();
  }
}
