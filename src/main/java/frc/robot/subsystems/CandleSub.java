package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;

import frc.lib.util.CTREConfigs;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CandleSub extends SubsystemBase {

    //Declare motors and sensors
    CANdle candle;
    
    public CandleSub() { //Subsystem constructor
        //Initialize motors and sensors
        candle = new CANdle(Constants.CandleSub.candleID);
        candle.configBrightnessScalar(Constants.CandleSub.brightness);
        candle.configLEDType(LEDStripType.GRB);
        candle.configStatusLedState(true);
        setLEDsCyan();
    }

    //Declare subsystem methods
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

    @Override //This method is called continuously
    public void periodic() {}

    @Override //This method is called continuously during simulation
    public void simulationPeriodic() {}
}
