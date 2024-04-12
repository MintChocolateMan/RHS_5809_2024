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
    
    private boolean intaked;

    public IntakeSub() { 
        intakeMotor = new TalonFX(Constants.IntakeSub.intakeMotorID);
        intakeMotor.getConfigurator().apply(Robot.ctreConfigs.intakeFXConfig);
        intakeMotor.setInverted(Constants.IntakeSub.intakeMotorReversed);
        intakeMotor.setPosition(Constants.IntakeSub.intakePIDGoal);
        intakeLineBreaker = new DigitalInput(Constants.IntakeSub.intakeLineBreakerID);
        shooterLineBreaker = new DigitalInput(Constants.IntakeSub.shooterLineBreakerID);

        intakeOverride = false;

        intaked = false;
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
        if (getIntakeLineBreaker() == false && getShooterLineBreaker() == false) {
            intaked = false;
        } else if (getIntakeLineBreaker() == false && getShooterLineBreaker() == true) {
            intaked = true;
        } 
        if (getIntakeLineBreaker() == false && getShooterLineBreaker() == false) {
            intakeMotor.setVoltage(0);
        } else if (intaked == false && getIntakeLineBreaker() == true) {
            intakeMotor.setVoltage(12);
        } else if (intaked == false && getShooterLineBreaker() == true) {
            intakeMotor.setVoltage(6);
        } else if (intaked == true && getIntakeLineBreaker() == false) {
            intakeMotor.setVoltage(-1);
        } else if (intaked == true && getIntakeLineBreaker() == true) {
            intakeMotor.setVoltage(0);
        }


        /*if (getIntakeLineBreaker() == false && getShooterLineBreaker() == false) {
            intakeMotor.setVoltage(0);
            intaked = false;
        }
        else if (getIntakeLineBreaker() == true && getShooterLineBreaker() == false) intakeMotor.setVoltage(12);
        else if (getIntakeLineBreaker() == true && getShooterLineBreaker() == true) intakeMotor.setVoltage(2);
        else if (getIntakeLineBreaker() == false && getShooterLineBreaker() == true) intakeMotor.setVoltage(-2);*/

        /*if (getIntakeLineBreaker() == false && getShooterLineBreaker() == false) intaked = false;
        else if (getIntakeLineBreaker() == false && getShooterLineBreaker() == true) intaked = true;
        else if (intaked == false && getIntakeLineBreaker() == true) intakeMotor.setVoltage(12);
        else if (intaked == true && getIntakeLineBreaker() == false) intakeMotor.setVoltage(-2);
        else if (intaked == true && getIntakeLineBreaker() == true) intakeMotor.stopMotor();
        else intakeMotor.stopMotor();*/

        /*if (getShooterLineBreaker() && getIntakeLineBreaker()) intakeMotor.setVoltage(1);
        else if (getIntakeLineBreaker()) intakeMotor.setVoltage(6);
        else intakeMotorOffDefault();*/
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
