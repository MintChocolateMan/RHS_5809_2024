package frc.robot.autoCommands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class OFSsetPoseFarStageNote extends Command {
  
    private final PoseEstimatorSub poseEstimatorSub;

    private final Translation2d blueTranslation = new Translation2d(8.2, 5.8);

    public OFSsetPoseFarStageNote(PoseEstimatorSub poseEstimatorSub) { 
  
        this.poseEstimatorSub = poseEstimatorSub;

        addRequirements(poseEstimatorSub);
    }

    @Override 
    public void initialize() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            poseEstimatorSub.setPoseTranslation(new Translation2d(16.5 - blueTranslation.getX(), blueTranslation.getY()));
        } else {
            poseEstimatorSub.setPoseTranslation(blueTranslation);
        }
    }

    @Override 
    public void execute() {}

    @Override 
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return true;
    }
}
