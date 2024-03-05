package frc.robot.subsystems;

import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class PneumaticSub extends SubsystemBase {

    //Declare motors and sensors
    private PneumaticHub pneumaticHub;
    private DoubleSolenoid leftClimber;
    private DoubleSolenoid rightClimber;
    
    public PneumaticSub() { //Subsystem constructor
        //Initialize motors and sensors
        pneumaticHub = new PneumaticHub(Constants.PneumaticSub.pneumaticHubID);
        
        leftClimber = pneumaticHub.makeDoubleSolenoid(Constants.PneumaticSub.leftClimberForwardID, Constants.PneumaticSub.leftClimberReverseID);
        rightClimber = pneumaticHub.makeDoubleSolenoid(Constants.PneumaticSub.rightClimberForwardID, Constants.PneumaticSub.rightClimberReverseID);

        leftClimber.set(Value.kOff);
        rightClimber.set(Value.kOff);
    }

    //Declare methods
    public void climbersUp() {
        leftClimber.set(Value.kForward);
        rightClimber.set(Value.kForward);
    }

    public void climbersDown() {
        leftClimber.set(Value.kReverse);
        rightClimber.set(Value.kReverse);
    }

    public void climbersOff() {
        leftClimber.set(Value.kOff);
        rightClimber.set(Value.kOff);
    }

    //Declare inline Commands
    public Command ExampleInlineCommand() {
        return runOnce(() -> {
        });
    }

    @Override //This method is called continuously
    public void periodic() {}

    @Override //This method is called continuously during simulation
    public void simulationPeriodic() {}
}
