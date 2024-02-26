package frc.robot.subsystems;

import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.DoubleSolenoid;

public class PneumaticSub extends SubsystemBase {

    //Declare motors and sensors
    PneumaticHub pneumaitcHub;
    DoubleSolenoid leftClimber;
    DoubleSolenoid rightClimber;
    
    public PneumaticSub() { //Subsystem constructor
        //Initialize motors and sensors
        PneumaticHub pneumaticHub = new PneumaticHub(Constants.PneumaticSub.pneumaticHubID);
        DoubleSolenoid leftClimber = pneumaticHub.makeDoubleSolenoid(Constants.PneumaticSub.leftClimberForwardID, Constants.PneumaticSub.leftClimberReverseID);
        DoubleSolenoid rightClimber = pneumaticHub.makeDoubleSolenoid(Constants.PneumaticSub.rightClimberForwardID, Constants.PneumaticSub.rightClimberReverseID);
        climbersDown();
    }

    //Declare subsystem suppliers

    //Declare methods
    public void climbersUp() {
        leftClimber.set(DoubleSolenoid.Value.kForward);
        rightClimber.set(DoubleSolenoid.Value.kForward);
    }

    public void climbersDown() {
        leftClimber.set(DoubleSolenoid.Value.kReverse);
        rightClimber.set(DoubleSolenoid.Value.kReverse);
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
