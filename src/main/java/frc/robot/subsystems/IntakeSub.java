package frc.robot.subsystems;

import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.hardware.TalonFX;

public class IntakeSub extends SubsystemBase {

    private TalonFX intakeMotor;
    private DigitalInput lineBreaker;

    private CandleSub candleSub;
    
    public IntakeSub(CandleSub candleSub) { 
        intakeMotor = new TalonFX(Constants.IntakeSub.intakeMotorID);
        intakeMotor.setInverted(Constants.IntakeSub.intakeMotorReversed);
        intakeMotor.setPosition(Constants.IntakeSub.intakePIDGoal);
        lineBreaker = new DigitalInput(Constants.IntakeSub.lineBreakerID);

        this.candleSub = candleSub;
    }

    public boolean getNoteLoaded() {
        return lineBreaker.get();
    }

    public void updateLEDs() {
        if (getNoteLoaded()) {
            candleSub.setLEDsGreen();
        } else candleSub.setLEDsAlliance();
    }

    public void intakeMotorOn() {
        intakeMotor.set(Constants.IntakeSub.intakeMotorSpeed);
    }

    public void intakeMotorReverse() {
        intakeMotor.set(-Constants.IntakeSub.intakeMotorReverseSpeed);
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

        SmartDashboard.putBoolean("NOTE LOADED", !getNoteLoaded());
    }

    @Override 
    public void simulationPeriodic() {}
}
