package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Robot;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.hardware.TalonFX;

public class UpdatedIntakeSub extends SubsystemBase {

    public TalonFX intakeMotor;
    private DigitalInput lineBreaker;

    private CandleSub candleSub;
    
    public UpdatedIntakeSub(CandleSub candleSub) { 
        intakeMotor = new TalonFX(Constants.IntakeSub.intakeMotorID);
        intakeMotor.getConfigurator().apply(Robot.ctreConfigs.intakeFXConfig);
        intakeMotor.setInverted(Constants.IntakeSub.intakeMotorReversed);
        intakeMotor.setPosition(Constants.IntakeSub.intakePIDGoal);
        lineBreaker = new DigitalInput(Constants.IntakeSub.lineBreakerID);

        this.candleSub = candleSub;
    }

    public boolean getNoteLoaded() {
        if (lineBreaker.get()) {
            return false;
        } else return true;
    }

    public void updateLEDs() {
        if (getNoteLoaded() == true) {
            candleSub.setLEDsGreen();
        } else candleSub.setLEDsCyan();
    }

    public void intakeMotorOn() {
        intakeMotor.set(Constants.IntakeSub.intakeMotorSpeed);
    }

    public void intakeMotorReverse() {
        intakeMotor.set(-Constants.IntakeSub.intakeMotorReverseSpeed);
    }

    public void intakeMotorSlow() {
        intakeMotor.set(Constants.IntakeSub.intakeMotorReverseSpeed);
    }

    public void intakeMotorOff() {
        intakeMotor.set(0);
    }

    public double getIntakeMotorPosition() {
        return intakeMotor.getPosition().getValueAsDouble();
    }

    public void resetIntakeMotorPosition() {
        intakeMotor.setPosition(0);
    }

    public void intakeMotorToPID() {
        intakeMotor.set(Constants.IntakeSub.intakePID.calculate(getIntakeMotorPosition(), Constants.IntakeSub.intakePIDGoal));
    }

    @Override 
    public void periodic() {
        updateLEDs();

        SmartDashboard.putBoolean("Note Loaded", getNoteLoaded());
    }

    @Override 
    public void simulationPeriodic() {}
}
