package org.firstinspires.ftc.teamcode.mechwarriors.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.mechwarriors.SignalSide;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

public class AprilTagSignalDetector implements SignalDetector {
    @Override
    public SignalSide detect() {
        return null;
    }
//    private Telemetry telemetry;
//    private HardwareMap hardwareMap;
//    private OpenCvCamera camera;
//    private AprilTagDetectionPipeline aprilTagDetectionPipeline;
//    private AprilTagDetection tagOfInterest = null;
//    private SignalSide detectedSignalSide = null;
//
//    static final double FEET_PER_METER = 3.28084;
//
//    // Lens intrinsics
//    // UNITS ARE PIXELS
//    // NOTE: this calibration is for the C270 webcam at 640x480
//    double fx = 822.317;
//    double fy = 822.317;
//    double cx = 319.495;
//    double cy = 242.502;
//
//    final int RESOLUTION_WIDTH = 640;
//    final int RESOLUTION_HEIGHT = 480;
//
//    // UNITS ARE METERS
//    final double TAG_SIZE = 0.03;
//
//    final int ID_TAG_OF_INTEREST_1 = 300; // Tag ID 300 from the 36h11 family
//    final int ID_TAG_OF_INTEREST_2 = 301; // Tag ID 301 from the 36h11 family
//    final int ID_TAG_OF_INTEREST_3 = 302; // Tag ID 302 from the 36h11 family
//
//    public AprilTagSignalDetector(Telemetry telemetry, HardwareMap hardwareMap) {
//        this.telemetry = telemetry;
//        this.hardwareMap = hardwareMap;
//
//        init();
//    }
//
//    @Override
//    public SignalSide detect() {
//        ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
//
//        if (currentDetections.size() != 0) {
//            boolean tagFound = false;
//
//            for (AprilTagDetection tag : currentDetections) {
//                if (tag.id == ID_TAG_OF_INTEREST_1) {
//                    tagOfInterest = tag;
//                    detectedSignalSide = SignalSide.ONE;
//                    tagFound = true;
//                    break;
//                } else if (tag.id == ID_TAG_OF_INTEREST_2) {
//                    tagOfInterest = tag;
//                    detectedSignalSide = SignalSide.TWO;
//                    tagFound = true;
//                    break;
//                } else if (tag.id == ID_TAG_OF_INTEREST_3) {
//                    tagOfInterest = tag;
//                    detectedSignalSide = SignalSide.THREE;
//                    tagFound = true;
//                    break;
//                }
//            }
//
//            /*
//            if (tagFound) {
//                telemetry.addLine("Tag of interest is in sight.\n\nLocation data:");
//                tagToTelemetry(tagOfInterest);
//            } else {
//                telemetry.addLine("Don't see tag of interest ");
//
//                if (tagOfInterest == null) {
//                    telemetry.addLine("(The tag has never been seen)");
//                } else {
//                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
//                    tagToTelemetry(tagOfInterest);
//                }
//            }
//            */
//        } else {
//            detectedSignalSide = SignalSide.NONE;
//            /*
//            telemetry.addLine("Don't see tag of interest");
//
//            if (tagOfInterest == null) {
//                telemetry.addLine("(The tag has never been seen)");
//            } else {
//                telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
//                tagToTelemetry(tagOfInterest);
//            }
//            */
//        }
//        return detectedSignalSide;
//    }
//
//    private void init() {
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(TAG_SIZE, fx, fy, cx, cy);
//
//        camera.setPipeline(aprilTagDetectionPipeline);
//        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            @Override
//            public void onOpened() {
//                camera.startStreaming(RESOLUTION_WIDTH, RESOLUTION_HEIGHT, OpenCvCameraRotation.UPRIGHT);
//            }
//
//            @Override
//            public void onError(int errorCode) {
//
//            }
//        });
//    }
//
//    void tagToTelemetry(AprilTagDetection detection) {
//        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
//        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x * FEET_PER_METER));
//        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y * FEET_PER_METER));
//        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z * FEET_PER_METER));
//        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
//        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
//        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
//    }
}
