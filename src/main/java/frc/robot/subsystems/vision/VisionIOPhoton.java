package frc.robot.subsystems.vision;

import com.spamrobotics.vision.SimCamera;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.LimelightHelpers.PoseEstimate;

public class VisionIOPhoton implements VisionIO {

    final SimCamera camera;

    public VisionIOPhoton() {
        camera = new SimCamera(
            "DriveCamera",
            () -> RobotContainer.instance.drivetrain.getSimPose(),
            VisionSubsystem.TURRET_TO_CAMERA,
            Robot.aprilTagFieldLayout
        );
    }

    @Override
    public void update(VisionInputs inputs) {
        if (Robot.isSimulation()) {
            camera.update();
            PoseEstimate estimate = camera.getPoseEstimate();
            inputs.turretPoseEstimate = estimate;
            inputs.turretPoseEstimateMT2 = estimate;
            inputs.turretCameraConnected = true;
        }
    }
}
