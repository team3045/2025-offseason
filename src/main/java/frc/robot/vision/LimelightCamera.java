package frc.robot.vision;

import edu.wpi.first.math.VecBuilder;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.CommandSwerveDrivetrain;

public class LimelightCamera {
    private static CommandSwerveDrivetrain drivetrain = RobotContainer.drivetrain;
    private String cameraName;

    public LimelightCamera(int cameraSettingsIndex) {
        cameraName = VisionConstants.cameraNames[cameraSettingsIndex];
        LimelightHelpers.setCameraPose_RobotSpace(
            VisionConstants.cameraNames[cameraSettingsIndex],
            VisionConstants.cameraPoses[cameraSettingsIndex].getTranslation().getX(),
            VisionConstants.cameraPoses[cameraSettingsIndex].getTranslation().getY(),
            VisionConstants.cameraPoses[cameraSettingsIndex].getTranslation().getZ(),
            VisionConstants.cameraPoses[cameraSettingsIndex].getRotation().getZ(),
            VisionConstants.cameraPoses[cameraSettingsIndex].getRotation().getY(),
            VisionConstants.cameraPoses[cameraSettingsIndex].getRotation().getX()
        );
    }

    public void gather() {
        LimelightHelpers.PoseEstimate poseEstimate;
        poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(cameraName);

        //We dont use vision for orientation
        if (poseEstimate != null && poseEstimate.tagCount > 0) {
            drivetrain.addVisionMeasurement(poseEstimate.pose, poseEstimate.timestampSeconds, VecBuilder.fill(0.1, 0.1, 99999));
        }
    }
}