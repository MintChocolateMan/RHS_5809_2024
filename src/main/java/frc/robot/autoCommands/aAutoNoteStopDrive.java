package frc.robot.autoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class aAutoNoteStopDrive extends Command {
  
    private final PoseEstimatorSub poseEstimatorSub;

    public aAutoNoteStopDrive(PoseEstimatorSub poseEstimatorSub) {
   
        this.poseEstimatorSub = poseEstimatorSub;

        addRequirements();
    }

    @Override 
    public void initialize() {}

    @Override 
    public void execute() {}

    @Override 
    public void end(boolean interrupted) {
        poseEstimatorSub.setAutoNoteSeen(false);
    }

    @Override 
    public boolean isFinished() {
        if (poseEstimatorSub.getAutoNoteSeen() == true) return true;
        else return false;
    }
}
