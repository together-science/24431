package org.firstinspires.ftc.teamcode.chassis;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

public class ScorpCamera {
    private VisionPortal visionPortal = null;               // Used to manage the video source.
    private AprilTagProcessor aprilTag = null;              // Used for managing the AprilTag detection process.
    private final LinearOpMode op;
    private WebcamName cam = null;

    public ScorpCamera(LinearOpMode op, String cameraName) {
        this.op = op;
        try {
            this.cam = op.hardwareMap.get(WebcamName.class, cameraName);
        } catch (Exception ignored) {
            return;
        }
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // e.g. Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(2);

        // Create the vision portal by using a builder.
        visionPortal = new VisionPortal.Builder()
                .setCamera(cam)
                .addProcessor(aprilTag)
                .build();
    }

    //
    // can we see one of the desired tags? if so, return detection
    //
    public void off(){
        visionPortal.stopStreaming();
    }
    public void on(){
        visionPortal.resumeStreaming();
    }
    public AprilTagDetection getDetection(final List<Integer> desiredTagIds) {
        if (this.cam == null) {
            return null;
        }
        // Step through the list of detected tags and look for a matching tag
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            // Look to see if we have size info on this tag.
            if (detection.metadata != null) {
                //  Check to see if we want to track towards this tag.
                if (desiredTagIds.contains(detection.id)) {
                    return detection;
                }
            }
        }
        return null;
    }

    /*
     Manually set the camera gain and exposure.
     This can only be called AFTER calling initAprilTag(), and only works for Webcams;
    */
    private void setManualExposure(int exposureMS, int gain) {
        if (this.cam == null) {
            return;
        }
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            op.telemetry.addData("Camera", "Waiting");
            op.telemetry.update();
            while (!op.isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                op.sleep(20);
            }
            op.telemetry.addData("Camera", "Ready");
            op.telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!op.isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                op.sleep(50);
            }
            exposureControl.setExposure(exposureMS, TimeUnit.MILLISECONDS);
            op.sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            op.sleep(20);
        }
    }
}
