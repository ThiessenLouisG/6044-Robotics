// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.DrivetrainConstants.*;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
// import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/* This class declares the subsystem for the robot drivetrain if controllers are connected via CAN. Make sure to go to
 * RobotContainer and uncomment the line declaring this subsystem and comment the line for PWMDrivetrain.
 *
 * The subsystem contains the objects for the hardware contained in the mechanism and handles low level logic
 * for control. Subsystems are a mechanism that, when used in conjuction with command "Requirements", ensure
 * that hardware is only being used by 1 command at a time.
 */
public class CANDrivetrain extends SubsystemBase {
  /*Class member variables. These variables represent things the class needs to keep track of and use between
  different method calls. */
  DifferentialDrive m_drivetrain;
  private final WPI_TalonSRX leftFront = new WPI_TalonSRX(kLeftFrontID);
  private final WPI_TalonSRX leftRear = new WPI_TalonSRX(kLeftRearID);
  private final WPI_TalonSRX rightFront = new WPI_TalonSRX(kRightFrontID);
  private final WPI_TalonSRX rightRear = new WPI_TalonSRX(kRightRearID);

  public static final double kMaxSpeed = 3.0; // meters per second
  public static final double kMaxAngularSpeed = 1 * Math.PI; // one rotation per second

  private final AnalogGyro m_gyro = new AnalogGyro(0);

  // private final PIDController m_leftPIDController = new PIDController(1, 0, 0);
  // private final PIDController m_rightPIDController = new PIDController(1, 0, 0);

  // private final DifferentialDriveKinematics m_kinematics =
  //     new DifferentialDriveKinematics(kTrackWidth);

  // private final DifferentialDriveOdometry m_odometry;

  // private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(1, 3);

  /*Constructor. This method is called when an instance of the class is created. This should generally be used to set up
   * member variables and perform any configuration or set up necessary on hardware.
   */
  public CANDrivetrain() {

    /*Sets current limits for the drivetrain motors. This helps reduce the likelihood of wheel spin, reduces motor heating
     *at stall (Drivetrain pushing against something) and helps maintain battery voltage under heavy demand */
    leftFront.configPeakCurrentLimit(kCurrentLimit);
    leftRear.configPeakCurrentLimit(kCurrentLimit);
    rightFront.configPeakCurrentLimit(kCurrentLimit);
    rightRear.configPeakCurrentLimit(kCurrentLimit);

    leftFront.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    leftFront.setSensorPhase(true);
    rightFront.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    rightFront.setSensorPhase(true);

    // Set the rear motors to follow the front motors.
    leftRear.follow(leftFront);
    rightRear.follow(rightFront);

    // Invert the left side so both side drive forward with positive motor outputs
    leftFront.setInverted(false);
    leftRear.setInverted(false);
    rightFront.setInverted(true);
    rightRear.setInverted(true);

    // Put the front motors into the differential drive object. This will control all 4 motors with
    // the rears set to follow the fronts
    m_drivetrain = new DifferentialDrive(leftFront, rightFront);

    m_gyro.reset();

    leftFront.setSelectedSensorPosition(0);
    rightFront.setSelectedSensorPosition(0);

    // m_odometry =
    //     new DifferentialDriveOdometry(
    //         m_gyro.getRotation2d(), getLeftEncoderDistance(), getRightEncoderDistance());
  }

  public void operatorDrive(double speed, double rot) {
    m_drivetrain.arcadeDrive(speed, rot);
  }

  // public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
  //   final double leftFeedforward = m_feedforward.calculate(speeds.leftMetersPerSecond);
  //   final double rightFeedforward = m_feedforward.calculate(speeds.rightMetersPerSecond);

  //   final double leftOutput =
  //       m_leftPIDController.calculate(
  //           leftFront.getSelectedSensorVelocity(), speeds.leftMetersPerSecond);
  //   final double rightOutput =
  //       m_rightPIDController.calculate(
  //           rightFront.getSelectedSensorVelocity(), speeds.rightMetersPerSecond);
  //   leftFront.setVoltage(leftOutput + leftFeedforward);
  //   rightFront.setVoltage(rightOutput + rightFeedforward);

  //   leftFront.set(ControlMode.Velocity, leftFeedforward);
  //   rightFront.set(ControlMode.Velocity, rightFeedforward);
  // }

  /*Method to control the drivetrain using arcade drive. Arcade drive takes a speed in the X (forward/back) direction
   * and a rotation about the Z (turning the robot about it's center) and uses these to control the drivetrain motors */
  public void arcadeDrive(double speed, double rotation) {
    m_drivetrain.arcadeDrive(speed, rotation);
  }

  // public void drive(double xSpeed, double rot) {
  //   var wheelSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, rot));
  //   setSpeeds(wheelSpeeds);
  // }

  // public void updateOdometry() {
  //   m_odometry.update(m_gyro.getRotation2d(), getLeftEncoderDistance(),
  // getRightEncoderDistance());
  // }

  public double getLeftEncoderDistance() {
    return leftFront.getSelectedSensorPosition() / kEncoderResolution * 2 * Math.PI * kWheelRadius;
  }

  public double getRightEncoderDistance() {
    return rightFront.getSelectedSensorPosition() / kEncoderResolution * 2 * Math.PI * kWheelRadius;
  }

  @Override
  public void periodic() {
    /*This method will be called once per scheduler run. It can be used for running tasks we know we want to update each
     * loop such as processing sensor data. Our drivetrain is simple so we don't have anything to put here */
    SmartDashboard.putNumber("Left Drive", getLeftEncoderDistance());
    SmartDashboard.putNumber("Right Drive", getRightEncoderDistance());
  }
}
