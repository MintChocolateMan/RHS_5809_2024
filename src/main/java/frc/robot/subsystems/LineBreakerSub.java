package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class LineBreakerSub extends SubsystemBase{
    private DigitalInput lineBreaker;

    public LineBreakerSub() {
        lineBreaker = new DigitalInput(1);
    }

    public boolean getLineBreaker() {
        return lineBreaker.get();
    }


    @Override
    public void periodic() {
        SmartDashboard.putBoolean("LineBreaker: ", getLineBreaker());
    }
}
