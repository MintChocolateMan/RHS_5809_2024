package frc.robot.autoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class aAutoNoteDetected extends Command {
  
    private final PoseEstimatorSub poseEstimatorSub;

    public aAutoNoteDetected(PoseEstimatorSub poseEstimatorSub) {
   
        this.poseEstimatorSub = poseEstimatorSub;

        addRequirements();
    }

    @Override 
    public void initialize() {}

    @Override 
    public void execute() {}

    @Override 
    public void end(boolean interrupted) {
        poseEstimatorSub.setAutoNoteSeen(true);
    }

    @Override 
    public boolean isFinished() {
        if (poseEstimatorSub.getValidNote() == true) return true;
        else return false;
    }
}
