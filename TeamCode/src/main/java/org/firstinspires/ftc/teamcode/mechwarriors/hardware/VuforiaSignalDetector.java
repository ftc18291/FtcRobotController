package org.firstinspires.ftc.teamcode.mechwarriors.hardware;

import java.util.List;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.mechwarriors.SignalSide;

public class VuforiaSignalDetector implements SignalDetector {
    @Override
    public SignalSide detect() {
        return null;
    }

//    private SignalSide detectedSignalSide = null;
//
//    private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";
//
//    private static final String[] LABELS = {
//            "1 Bolt",
//            "2 Bulb",
//            "3 Panel"
//    };
//
//    private static final String VUFORIA_KEY =
//            "AWxXVvb/////AAABmfLOIRj/gkVCr0/M8YRLx+FbSVAoWdmjYSjL37w2Mo1AzGHZATUql6qYWlSdrvHlbcRi9J62kElhwXiFEp43l1k0XvyRwFLNIeNI/p5ldQ2lXikmPUQtM7atRdByHesdFFuLYvmzroDdSN9Y477hRgXl2pevCZTVx/eG66KfxhC1+LCxvwy4rRiAhET5s0XuofoItmQHz7T9p5CQRo+Lo3tJ5S8DTJBRLoMhPqlWaSt8ujsTS0jQskL/5WQ46MnbeJc+IK5fK6RxsHaztN8tY/5tPNgvGe0XPOOU2IRB/TtThZYGmgPmoxC5+RGabJyMzoPoXusUIzyFr+kFccvd3JcOm9o4irkYVa+7k1gJIwvA";
//
//    /**
//     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
//     * localization engine.
//     */
//    private VuforiaLocalizer vuforia;
//
//    /**
//     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
//     * Detection engine.
//     */
//    private TFObjectDetector tfod;
//
//    private HardwareMap hardwareMap;
//    private Telemetry telemetry;
//
//    public VuforiaSignalDetector(Telemetry telemetry, HardwareMap hardwareMap) {
//        this.telemetry = telemetry;
//        this.hardwareMap = hardwareMap;
//
//        initVuforia();
//        initTfod();
//
//        if (tfod != null) {
//            tfod.activate();
//            tfod.setZoom(1.0, 16.0 / 9.0);
//        }
//    }
//
//    public SignalSide detect() {
//        // telemetry.addLine("In detect");
//
//        if (tfod != null) {
//            // getUpdatedRecognitions() will return null if no new information is available since
//            // the last time that call was made.
//            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
//            if (updatedRecognitions != null) {
//                //telemetry.addData("# Objects Detected", updatedRecognitions.size());
//
//                if (updatedRecognitions.size() > 0) {
//                    Recognition recognition = updatedRecognitions.get(0);
//                    //telemetry.addData("recognition", recognition.getLabel());
//                    if (recognition.getLabel().equals(LABELS[0])) {
//                        detectedSignalSide = SignalSide.ONE;
//                    } else if (recognition.getLabel().equals(LABELS[1])) {
//                        detectedSignalSide = SignalSide.TWO;
//                    } else if (recognition.getLabel().equals(LABELS[2])) {
//                        detectedSignalSide = SignalSide.THREE;
//                    }
//                } else {
//                    detectedSignalSide = SignalSide.NONE;
//                    //telemetry.addLine("No signal detected");
//                }
//
//                // step through the list of recognitions and display image position/size information for each one
//                // Note: "Image number" refers to the randomized image orientation/number
//                /*for (Recognition recognition : updatedRecognitions) {
//                    double col = (recognition.getLeft() + recognition.getRight()) / 2 ;
//                    double row = (recognition.getTop()  + recognition.getBottom()) / 2 ;
//                    double width  = Math.abs(recognition.getRight() - recognition.getLeft()) ;
//                    double height = Math.abs(recognition.getTop()  - recognition.getBottom()) ;
//
//                    telemetry.addData(""," ");
//                    telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100 );
//                    telemetry.addData("- Position (Row/Col)","%.0f / %.0f", row, col);
//                    telemetry.addData("- Size (Width/Height)","%.0f / %.0f", width, height);
//                }*/
//                //telemetry.update();
//            }
//        }
//        return detectedSignalSide;
//    }
//
//    /**
//     * Initialize the Vuforia localization engine.
//     */
//    private void initVuforia() {
//        /*
//         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
//         */
//        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
//
//        parameters.vuforiaLicenseKey = VUFORIA_KEY;
//        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
//
//        //  Instantiate the Vuforia engine
//        vuforia = ClassFactory.getInstance().createVuforia(parameters);
//    }
//
//    /**
//     * Initialize the TensorFlow Object Detection engine.
//     */
//    private void initTfod() {
//        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
//                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
//        tfodParameters.minResultConfidence = 0.75f;
//        tfodParameters.isModelTensorFlow2 = true;
//        tfodParameters.inputSize = 300;
//        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
//
//        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
//        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
//        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
//        // tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
//    }
}