package org.firstinspires.ftc.teamcode.mechwarriors.opmodes;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.mechwarriors.hardware.ArtifactIntaker;
import org.firstinspires.ftc.teamcode.mechwarriors.hardware.ArtifactLauncher;
import org.firstinspires.ftc.teamcode.mechwarriors.hardware.ArtifactSorter;
import org.firstinspires.ftc.teamcode.mechwarriors.hardware.ArtifactSorterMode;
import org.firstinspires.ftc.teamcode.mechwarriors.hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.mechwarriors.hardware.LEDIndicator;

import java.util.List;

@Config
@TeleOp
public class DecodeTeleOpMode extends OpMode {
    private ColorSensor colorSensor;
    private DistanceSensor distanceSensor;

    private Servo light;

    LEDIndicator leftLedIndicatorLight;
    LEDIndicator rightLedIndicatorLight;

    public static double rampPosition = 0.25;

    Limelight3A limelight;

    private DriveTrain drivetrain;
    public double launcherArmPosition = 0.0;

    //DcMotorEx launcherMotor;
    //Servo launcherServo;

    DcMotorEx sorterMotor;

    Boolean sorterRotateButtonPressed = false;

    Boolean sorterReverseButtonPressed = false;

    Boolean sorterSetIntakeModeButtonPressed = false;
    Boolean sorterSetLaunchModeButtonPressed = false;


    DigitalChannel sorterTouchSensor;

    ArtifactIntaker artifactIntaker;
    ArtifactSorter artifactSorter;
    ArtifactLauncher artifactLauncher;

    HuskyLens huskyLens;


    @Override
    public void init() {


       // huskyLens = hardwareMap.get(HuskyLens.class, "huskyLens");
        leftLedIndicatorLight = new LEDIndicator(hardwareMap, "leftDirectionIndicator");
        rightLedIndicatorLight = new LEDIndicator(hardwareMap, "rightDirectionIndicator");

        //colorSensor = hardwareMap.get(ColorSensor.class, "artifactColorSensor");

        //light = hardwareMap.get(Servo.class, "light");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        drivetrain = new DriveTrain(hardwareMap, telemetry);

        // Artifact Intake
        artifactIntaker = new ArtifactIntaker(hardwareMap);

        // Artifact Sorter
        artifactSorter = new ArtifactSorter(hardwareMap, telemetry);

        sorterTouchSensor = hardwareMap.get(DigitalChannel.class, "sorterTouchSensor");

        // Artifact Launcher
//        launcherMotor = hardwareMap.get(DcMotorEx.class, "launcherMotor");
//        launcherMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        launcherServo = hardwareMap.get(Servo.class, "launcherServo");
//        launcherServo.scaleRange(0.65, 1.0);
//        launcherServo.setPosition(0);
        artifactLauncher = new ArtifactLauncher(hardwareMap, telemetry);

    }

    @Override
    public void start() {
        limelight.start();

        artifactSorter.init();
    }


    @Override
    public void loop() {
        artifactLauncher.checkSpeed();

        artifactLauncher.setRampPosition(rampPosition);

        double distance;

        LLResult result = limelight.getLatestResult();
        List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
        if (!fiducialResults.isEmpty()) {
            LLResultTypes.FiducialResult aprilTagResult = fiducialResults.get(0);
            int id = aprilTagResult.getFiducialId();
            telemetry.addData("goal", id);
            telemetry.addData("goal x", aprilTagResult.getTargetXDegrees());
            if (aprilTagResult.getTargetXDegrees() > 1.0) {
                // LEFT
                rightLedIndicatorLight.setColor(LEDIndicator.LEDColor.RED);
                leftLedIndicatorLight.setColor(LEDIndicator.LEDColor.GREEN);
            } else if (aprilTagResult.getTargetXDegrees() < -1.0) {
                // RIGHT
                rightLedIndicatorLight.setColor(LEDIndicator.LEDColor.GREEN);
                leftLedIndicatorLight.setColor(LEDIndicator.LEDColor.RED);
            } else if (aprilTagResult.getTargetXDegrees() <= 1.0 && aprilTagResult.getTargetXDegrees() >= -1.0) {
                // CENTER
                rightLedIndicatorLight.setColor(LEDIndicator.LEDColor.GREEN);
                leftLedIndicatorLight.setColor(LEDIndicator.LEDColor.GREEN);
            }
        } else {
            rightLedIndicatorLight.setColor(LEDIndicator.LEDColor.OFF);
            leftLedIndicatorLight.setColor(LEDIndicator.LEDColor.OFF);
        }

//        HuskyLens.Block[] blocks = huskyLens.blocks();
//        if (blocks.length > 0) {
//
//            telemetry.addData("Block count", blocks.length);
//            for (int i = 0; i < blocks.length; i++) {
//                if (blocks[i].id == 1) {
//                    telemetry.addLine("Purple ball spotted");
//                }
//                if (blocks[i].id == 2) {
//                    telemetry.addLine("Green ball spotted");
//                }
//            }
//        }

//        telemetry.addData("Red", colorSensor.red());
//        telemetry.addData("Blue", colorSensor.blue());
//        telemetry.addData("Green", colorSensor.green());
//        telemetry.addData("ARGB", colorSensor.argb());
//        telemetry.addData("Distance", distanceSensor.getDistance(DistanceUnit.CM));
        telemetry.addData("BallColor", determineColor());
        // telemetry.addData("motorPosition", sorterMotor.getCurrentPosition());

        //distance = getDistanceFromTag(llResult.getTa());
        //telemetry.addData("Distance", distance);
        //telemetry.addData("Target x", llResult.getTx() );
        //telemetry.addData();
        //telemetry.addData();


//        String color = determineColor();
//
//        switch (color) {
//            case "Green":
//                indicatorLight.setColor(IndicatorLightColor.GREEN);
//                break;
//            case "Purple":
//                indicatorLight.setColor(IndicatorLightColor.PURPLE);
//                break;
//        }

        double y = gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;
        drivetrain.drive(x, y, rx);

        if (gamepad1.left_bumper) {
            drivetrain.setSlowMode(true);
        } else if (gamepad1.right_bumper) {
            drivetrain.setSlowMode(false);
        }

        telemetry.addData("gamepad2.right_stick_y", gamepad2.right_stick_y);
      /*  if (gamepad2.right_stick_y > 0.5) {
            artifactLauncher.raiseLifter();
        } else if (gamepad2.right_stick_y < 0.5) {
            artifactLauncher.lowerLifter();
        }*/
        telemetry.addData("launcherArmPosition", launcherArmPosition);
        if (gamepad2.shareWasReleased()) {
            artifactLauncher.setShootingMode();
//            launcherArmPosition += 0.05 ;
//            if (launcherArmPosition > 0.6){
//                launcherArmPosition = 0;
//            }
//            artifactLauncher.setPosition(launcherArmPosition);
        }



        if (gamepad1.x) {
            drivetrain.setSuperSlowMode(true);
        } else if (gamepad1.y) {
            drivetrain.setSuperSlowMode(false);
        }

        // Intake motor
        if (gamepad2.b &&
               // artifactIntaker.getSweeperPosition() == 0 &&
                artifactSorter.getArtifactSorterMode() == ArtifactSorterMode.INTAKE) {
            artifactIntaker.runIntakeMotor();
        } else if (gamepad2.x &&
                artifactSorter.getArtifactSorterMode() == ArtifactSorterMode.INTAKE) {
            artifactIntaker.reverseIntakeMotor();
        } else {
            artifactIntaker.stopIntakeMotor();
        }

        // Intake sweepers
        if (gamepad2.dpad_down)
            artifactIntaker.setSweeperToRearPosition();
        if (gamepad2.dpad_up) {
            artifactIntaker.setSweeperToFrontPosition();
        }

        telemetry.addData("sorterTouchSensor", getSorterSensorState());
        telemetry.addData("artifactSorterMode", artifactSorter.getArtifactSorterMode());
        telemetry.addData("currentTicksTarget", artifactSorter.getCurrentTicksTarget());

        // Rotate sorter one slot
        if (gamepad2.left_trigger > 0.5) {
            sorterRotateButtonPressed = true;
        }
        if (gamepad2.left_trigger < 0.5 && sorterRotateButtonPressed) {
            sorterRotateButtonPressed = false;
            artifactSorter.rotateOneSlot();
        }

        if (gamepad2.y) {
            artifactSorter.sorterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            artifactSorter.sorterMotor.setPower(-0.3);
        } else if (gamepad2.yWasReleased()) {
            artifactSorter.sorterMotor.setPower(0);
            artifactSorter.sorterMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            artifactSorter.sorterMotor.setPower(0.3);
        }

        // Rotate sorter from launch to intake position
        if (gamepad2.left_bumper) {
            sorterSetIntakeModeButtonPressed = true;
        } else if (!gamepad2.left_bumper && sorterSetIntakeModeButtonPressed) {
            sorterSetIntakeModeButtonPressed = false;
            artifactSorter.setArtifactSorterMode(ArtifactSorterMode.INTAKE);
        }
        // Rotate sorter from intake to launch position
        if (gamepad2.right_bumper) {
            sorterSetLaunchModeButtonPressed = true;
        } else if (!gamepad2.right_bumper && sorterSetLaunchModeButtonPressed) {
            sorterSetLaunchModeButtonPressed = false;
            artifactSorter.setArtifactSorterMode(ArtifactSorterMode.LAUNCH);
        }

        // Launcher
        telemetry.addData("gamepad2.right_trigger", gamepad2.right_trigger);
        if (gamepad2.right_trigger > 0.5) {
            telemetry.addLine("right trigger pressed - start launch spinner motor");
            artifactLauncher.startFlywheel();
        } else {
            telemetry.addLine("stop launch spinner motor");
            artifactLauncher.stopFlywheel();
        }
        telemetry.addData("launcherMotor velocity", artifactLauncher.getLaunchMotorVelocity());

        if (gamepad2.a && artifactSorter.getArtifactSorterMode() == ArtifactSorterMode.LAUNCH) {
            artifactLauncher.launch();
        } else {
            artifactLauncher.launchReset();
        }
    }

    @Override
    public void stop() {
        leftLedIndicatorLight.setColor(LEDIndicator.LEDColor.OFF);
        leftLedIndicatorLight.setColor(LEDIndicator.LEDColor.OFF);
        limelight.stop();
    }

    private boolean getSorterSensorState() {
        return !sorterTouchSensor.getState();
    }

    public String determineColor() {
//        int green = colorSensor.green();
//        int red = colorSensor.red();
//        if (red > 100 && green > 150) {
//            return "Purple";
//        } else {
//            return "Green";
//        }
        return null;
    }

    private double getDistanceFromTag(double y) {
        double a = 5208.601;
        double b = -2.008664;
        return Math.pow(y / a, 1.0 / b);
    }




}