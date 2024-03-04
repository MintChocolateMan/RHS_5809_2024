package frc.robot.backup;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class i_IntakeOuttake extends Command {
  
    //Declare subsystems
    private final IntakeSub intakeSub;

    public i_IntakeOuttake(IntakeSub intakeSub) { //Command constructor
        //Initialize subsystems
        this.intakeSub = intakeSub;

        //Add subsystem requirements
        addRequirements(intakeSub);
    }

    @Override //Called when the command is initially scheduled.
    public void initialize() {
        intakeSub.intakeMotorReverse();
    }

    @Override // Called every time the scheduler runs while the command is scheduled.
    public void execute() {}

    @Override // Called once the command ends or is interrupted.
    public void end(boolean interrupted) {
        intakeSub.intakeMotorOff();
    }

    @Override // Returns true when the command should end.
    public boolean isFinished() {
        return false;
    }
}
