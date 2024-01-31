package frc.robot.subsystems;

import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.TalonFX;

public class IntakeSub extends SubsystemBase {
  
    TalonFX intakeMotor;

    public IntakeSub() {
        intakeMotor = new TalonFX(Constants.IntakeSub.intakeMotorID);
        intakeMotor.setInverted(Constants.IntakeSub.intakeMotorReversed);
    }

    public void intakeMotorOn() {
        intakeMotor.set(Constants.IntakeSub.intakeMotorSpeed);
    }

    public void intakeMotorOff() {
        intakeMotor.set(0);
    }

    public Command IntakeOn() {
        return runOnce(() -> {
            intakeMotor.set(Constants.IntakeSub.intakeMotorSpeed);
        });
    }

    public Command IntakeOff() {
        return runOnce(() -> {
            intakeMotor.set(0);
        });
    }

  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
