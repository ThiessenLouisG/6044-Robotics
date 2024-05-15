package frc.robot.commands;

import static frc.robot.Constants.IntakeConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANIntake;

// import frc.robot.subsystems.PWMLauncher;

public class PrepareIntakeLaunch extends Command {
  CANIntake m_intake;

  /** Creates a new PrepareLaunch. */
  public PrepareIntakeLaunch(CANIntake intake) {
    // save the launcher system internally
    m_intake = intake;
    // indicate that this command requires the launcher system
    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Set launch wheel to speed, keep feed wheel at 0 to let launch wheel spin up
    // Set the wheels to launching speed
    m_intake.setIntake(kIntakePrepareLaunchSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // There is nothing we need this command to do on each iteration. You could remove this method
    // and the default blank method
    // of the base class will run.
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.setIntake(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Always return false so the command never ends on it's own. In this project we use a timeout
    // decorator on the command to end it.
    return false;
  }
}
