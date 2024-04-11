package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Robot;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.hardware.TalonFX;

public class IntakeSub extends SubsystemBase {

    public TalonFX intakeMotor;
    private DigitalInput intakeLineBreaker;
    private DigitalInput shooterLineBreaker;

    private boolean intakeOverride;
    
    public IntakeSub() { 
        intakeMotor = new TalonFX(Constants.IntakeSub.intakeMotorID);
        intakeMotor.getConfigurator().apply(Robot.ctreConfigs.intakeFXConfig);
        intakeMotor.setInverted(Constants.IntakeSub.intakeMotorReversed);
        intakeMotor.setPosition(Constants.IntakeSub.intakePIDGoal);
        intakeLineBreaker = new DigitalInput(Constants.IntakeSub.intakeLineBreakerID);
        shooterLineBreaker = new DigitalInput(Constants.IntakeSub.shooterLineBreakerID);

        intakeOverride = false;
    }

    public void setIntakeOverride(boolean intakeOverride) {
        this.intakeOverride = intakeOverride;
    }

    public void startIntakeOverride() {
        intakeOverride = true;
    }

    public void endIntakeOverride() {
        intakeOverride = false;
    }

    public boolean getIntakeLineBreaker() {
        if (intakeLineBreaker.get() == false) return true;
        else return false;
    }

    public boolean getShooterLineBreaker() {
        if (shooterLineBreaker.get() == false) return true;
        else return false;
    }

    public void intakeMotorOn() {
        startIntakeOverride();
        intakeMotor.set(Constants.IntakeSub.intakeMotorSpeed);
    }

    public void intakeMotorOnDefault() {
        intakeMotor.set(Constants.IntakeSub.intakeMotorSpeed);
    }

    public void intakeMotorReverse() {
        startIntakeOverride();
        intakeMotor.set(-Constants.IntakeSub.intakeMotorReverseSpeed);
    }

    public void intakeMotorReverseDefault() {
        intakeMotor.set(-Constants.IntakeSub.intakeMotorReverseSpeed);
    }

    public void intakeMotorSlow() {
        startIntakeOverride();
        intakeMotor.set(Constants.IntakeSub.intakeMotorReverseSpeed);
    }

    public void intakeMotorSlowDefault() {
        intakeMotor.set(Constants.IntakeSub.intakeMotorReverseSpeed);
    }

    public void intakeMotorOff() {
        endIntakeOverride();
        intakeMotor.set(0);
    }

    public void intakeMotorOffDefault() {
        intakeMotor.set(0);
    }

    public boolean getNoteLoaded() {
        if (getIntakeLineBreaker() || getShooterLineBreaker()) return true;
        else return false;
    }

    public void suckBack() {
        if (getShooterLineBreaker() && getIntakeLineBreaker()) intakeMotorSlowDefault();
        else if (getIntakeLineBreaker()) intakeMotorOnDefault();
        else intakeMotorOffDefault();
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
        if (intakeOverride == false) suckBack();

        SmartDashboard.putBoolean("Note Loaded", getNoteLoaded());
    }

    @Override 
    public void simulationPeriodic() {}
}
