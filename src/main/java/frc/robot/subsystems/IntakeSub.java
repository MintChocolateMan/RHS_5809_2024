package frc.robot.subsystems;

import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.DigitalInput;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.hardware.TalonFX;

public class IntakeSub extends SubsystemBase {

    private TalonFX intakeMotor;
    private DigitalInput lineBreaker;
    
    public IntakeSub() { 
        intakeMotor = new TalonFX(Constants.IntakeSub.intakeMotorID);
        intakeMotor.setInverted(Constants.IntakeSub.intakeMotorReversed);
        intakeMotor.setPosition(Constants.IntakeSub.intakePIDGoal);
        lineBreaker = new DigitalInput(Constants.IntakeSub.lineBreakerID);
    }

    public boolean getNoteLoaded() {
        return lineBreaker.get();
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
        //SmartDashboard.putBoolean("NOTE LOADED", getNoteLoaded());
    }

    @Override 
    public void simulationPeriodic() {}
}
