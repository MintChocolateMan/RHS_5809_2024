package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CandleSub extends SubsystemBase {

    CANdle candle;

    IntakeSub intakeSub;
    PoseEstimatorSub poseEstimatorSub;

    Timer intakeTimer;
    Timer blinkTimer;
    boolean blink;
    
    public CandleSub(IntakeSub intakeSub, PoseEstimatorSub poseEstimatorSub) { 

        candle = new CANdle(Constants.CandleSub.candleID);
        candle.configBrightnessScalar(Constants.CandleSub.brightness);
        candle.configLEDType(LEDStripType.GRB);
        candle.configStatusLedState(true);
        setLEDsCyan();

        this.intakeSub = intakeSub;
        this.poseEstimatorSub = poseEstimatorSub;

        blink = false;
        blinkTimer = new Timer();
        blinkTimer.reset();
        blinkTimer.start();
        intakeTimer = new Timer();
        intakeTimer.stop();
        intakeTimer.reset();
    }

    public void setLEDs(int r, int g, int b) {
        candle.setLEDs(r, g, b);
    }

    public void setLEDsCyan() {
        candle.setLEDs(0, 255, 255);
    }

    public void setLEDsGreen() {
        candle.setLEDs(0, 255, 0);
    }

    public void setLEDsAlliance() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            if (alliance.get() == DriverStation.Alliance.Red) {
                candle.setLEDs(255, 0, 0);
            } else if (alliance.get() == DriverStation.Alliance.Blue) {
                candle.setLEDs(0, 0, 255);
            }
        } else candle.setLEDs(0, 255, 255);
    }

    public void updateBlink() {
        if (blinkTimer.get() > .2 && blink == false) {
            blinkTimer.reset();
            blinkTimer.start();
            blink = true;
        } else if (blinkTimer.get() > .2 && blink == true) {
            blinkTimer.reset();
            blinkTimer.start();
            blink = false;
        } else return;
    }

    public void updateIntake() {
        if (intakeSub.getNoteLoaded() == false) {
            intakeTimer.stop();
            intakeTimer.reset();
        } else intakeTimer.start();
    }

    public void updateLEDs() {
        updateBlink();
        updateIntake();
        if (intakeSub.getNoteLoaded() == false && poseEstimatorSub.getValidNote() == false) {
            candle.setLEDs(31, 0, 255);
        } else if (intakeSub.getNoteLoaded() == false && poseEstimatorSub.getValidNote() == true) {
            candle.setLEDs(255, 166, 0);
        } else if (intakeTimer.get() != 0 && intakeTimer.get() <= 1) {
            candle.setLEDs(0, 255, 0);
        } else if (intakeTimer.get() > 1 && poseEstimatorSub.getTagCount() > 1) {
            candle.setLEDs(0, 255, 255);
        }
    }

    @Override 
    public void periodic() {
        updateLEDs();
    }

    @Override 
    public void simulationPeriodic() {}
}
