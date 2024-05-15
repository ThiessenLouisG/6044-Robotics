// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// import static frc.robot.Constants.DrivetrainConstants.kEncoderResolution;
import static frc.robot.Constants.LauncherConstants.*;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.BangBangController;
// import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CANLauncher extends SubsystemBase {
  private final CANSparkMax m_LeftlaunchWheel;
  private final CANSparkMax m_RightlaunchWheel;
  private final WPI_TalonSRX m_feedWheel;
  private final RelativeEncoder m_leftLaunchEncoder;
  private final RelativeEncoder m_rightLaunchEncoder;
  private final BangBangController controller;

  private final SparkMaxPIDController lController;
  private final SparkMaxPIDController rController;

  /** Creates a new Launcher. */
  public CANLauncher() {
    m_LeftlaunchWheel = new CANSparkMax(kLeftLauncherID, MotorType.kBrushless);
    m_RightlaunchWheel = new CANSparkMax(kRightLauncherID, MotorType.kBrushless);
    m_feedWheel = new WPI_TalonSRX(kFeederID);

    m_RightlaunchWheel.setInverted(true);

    m_leftLaunchEncoder = m_LeftlaunchWheel.getEncoder();
    m_rightLaunchEncoder = m_RightlaunchWheel.getEncoder();

    lController = m_LeftlaunchWheel.getPIDController();
    lController.setP(0.00055);
    lController.setI(2e-7);
    lController.setD(0.005);
    rController = m_RightlaunchWheel.getPIDController();
    rController.setP(0.00055);
    rController.setI(2e-7);
    rController.setD(0.005);

    controller = new BangBangController();

    m_LeftlaunchWheel.setSmartCurrentLimit(kLauncherCurrentLimit);
    m_RightlaunchWheel.setSmartCurrentLimit(kLauncherCurrentLimit);
  }

  /**
   * This method is an example of the 'subsystem factory' style of command creation. A method inside
   * the subsytem is created to return an instance of a command. This works for commands that
   * operate on only that subsystem, a similar approach can be done in RobotContainer for commands
   * that need to span subsystems. The Subsystem class has helper methods, such as the startEnd
   * method used here, to create these commands.
   */
  public Command getIntakeLauncherCommand() {
    // The startEnd helper method takes a method to call when the command is initialized and one to
    // call when it ends
    return this.startEnd(
        // When the command is initialized, set the wheels to the intake speed values
        () -> {
          setFeedWheel(kIntakeFeederSpeed);
          setLeftLaunchWheel(kIntakeLeftLauncherSpeed);
          setRightLaunchWheel(kIntakeRightLauncherSpeed);
        },
        // When the command stops, stop the wheels
        () -> {
          stop();
        });
  }

  // An accessor method to set the speed (technically the output percentage) of the launch wheel
  public void setLeftLaunchWheel(double speed) {
    // double calcSpeed = controller.calculate(m_leftLaunchEncoder.getVelocity(), speed);
    // m_LeftlaunchWheel.set(calcSpeed);

    lController.setReference(speed, ControlType.kVelocity);
  }

  public void setRightLaunchWheel(double speed) {
    // double calcSpeed = controller.calculate(m_rightLaunchEncoder.getVelocity(), speed);
    // m_RightlaunchWheel.set(calcSpeed);
    rController.setReference(speed, ControlType.kVelocity);
  }

  // An accessor method to set the speed (technically the output percentage) of the feed wheel
  public void setFeedWheel(double speed) {
    m_feedWheel.set(speed);
  }

  // A helper method to stop both wheels. You could skip having a method like this and call the
  // individual accessors with speed = 0 instead
  public void stop() {
    m_LeftlaunchWheel.set(0);
    m_RightlaunchWheel.set(0);
    m_feedWheel.set(0);
  }
}
