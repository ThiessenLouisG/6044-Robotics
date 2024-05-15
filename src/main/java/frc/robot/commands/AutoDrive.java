package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANDrivetrain;

public class AutoDrive extends Command {
  CANDrivetrain m_drivetrain;
  double m_speed;
  double m_rot;

  /** Creates a new LaunchNote. */
  public AutoDrive(CANDrivetrain drivetrain, double speed, double rot) {
    // save the launcher system internally
    m_drivetrain = drivetrain;
    m_speed = speed;
    m_rot = rot;
    // indicate that this command requires the launcher system
    addRequirements(m_drivetrain);
  }

  @Override
  public void initialize() {
    m_drivetrain.arcadeDrive(m_speed, m_rot);
  }

  @Override
  public void execute() {
    System.out.println("driving");
    m_drivetrain.arcadeDrive(m_speed, m_rot);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the wheels when the command ends.
    m_drivetrain.arcadeDrive(0, 0);
  }
}
