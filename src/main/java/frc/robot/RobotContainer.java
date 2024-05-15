// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
// import frc.robot.Constants.DrivetrainConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.Drive;
import frc.robot.commands.LaunchNote;
import frc.robot.commands.PrepareIntakeLaunch;
import frc.robot.commands.PrepareLaunch;
import frc.robot.subsystems.CANDrivetrain;
import frc.robot.subsystems.CANIntake;
import frc.robot.subsystems.CANLauncher;

// import static frc.robot.Constants.DrivetrainConstants.*;
// import frc.robot.subsystems.PWMDrivetrain;
// import frc.robot.subsystems.PWMLauncher;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems are defined here.
  private final CANDrivetrain m_drivetrain = new CANDrivetrain();
  private final CANLauncher m_launcher = new CANLauncher();
  private final CANIntake m_intake = new CANIntake();
  private final SendableChooser<Command> auto_selector = new SendableChooser();

  /*
   * The gamepad provided in the KOP shows up like an XBox controller if the mode
   * switch is set to X mode using the
   * switch on the top.
   */
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  // private final CommandXboxController m_operatorController =
  // new CommandXboxController(OperatorConstants.kOperatorControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    auto_selector.setDefaultOption(
        "shoot auto", Autos.shootAuto(m_drivetrain, m_launcher, m_intake));
    auto_selector.addOption(
        "shoot drive auto", Autos.driveShootAuto(m_drivetrain, m_launcher, m_intake));
    SmartDashboard.putData(auto_selector);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be accessed via the
   * named factory methods in the Command* classes in edu.wpi.first.wpilibj2.command.button (shown
   * below) or via the Trigger constructor for arbitary conditions
   */
  private void configureBindings() {
    // Set the default command for the drivetrain to drive using the joysticks

    // m_drivetrain.setDefaultCommand(
    // new RunCommand(
    // () -> m_drivetrain.drive(-m_driverController.getLeftY(),
    // -m_driverController.getLeftX()),
    // m_drivetrain));

    m_drivetrain.setDefaultCommand(new Drive(m_drivetrain, m_driverController));

    // new RunCommand(
    // () ->
    // m_drivetrain.drive(
    // -m_speedLimiter.calculate(m_driverController.getLeftY())
    // * CANDrivetrain.kMaxSpeed,
    // -m_rotLimiter.calculate(m_driverController.getLeftX())
    // * CANDrivetrain.kMaxAngularSpeed),
    // m_drivetrain));

    m_driverController.a().toggleOnTrue(new PrepareLaunch(m_launcher));

    m_driverController
        .b()
        .whileTrue(
            new PrepareIntakeLaunch(m_intake).withTimeout(.1).andThen(new LaunchNote(m_intake)));

    // Set up a binding to run the intake command while the operator is pressing and
    // holding the
    // left Bumper
    m_driverController.leftBumper().whileTrue(m_launcher.getIntakeLauncherCommand());

    m_driverController.rightTrigger().whileTrue(m_intake.getIntakeCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return auto_selector.getSelected();
    // return Autos.driveShootAuto(m_drivetrain, m_launcher, m_intake);
  }
}
