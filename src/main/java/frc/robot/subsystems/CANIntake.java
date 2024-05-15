package frc.robot.subsystems;

import static frc.robot.Constants.IntakeConstants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CANIntake extends SubsystemBase {
  private final CANSparkMax m_intake = new CANSparkMax(kIntakeID, MotorType.kBrushless);

  public CANIntake() {}

  public Command getIntakeCommand() {
    return this.startEnd(
        () -> {
          setIntake(kIntakeSpeed);
        },
        () -> {
          stop();
        });
  }

  public void setIntake(double speed) {
    m_intake.set(speed);
  }

  public void stop() {
    m_intake.set(0);
  }
}
