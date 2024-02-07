package frc.robot.subsystems;

//import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ExampleSub extends SubsystemBase {

    //Declare motors and sensors
    
    public ExampleSub() { //Subsystem constructor
        //Initialize motors and sensors
    }

    //Declare subsystem suppliers
    public boolean exampleSupplier() {
        return false;
    }

    //Declare methods
    public void exampleMethod() {}

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
