package org.firstinspires.ftc.teamcode.mechwarriors.opmodes;


import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
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
import org.firstinspires.ftc.teamcode.mechwarriors.hardware.AutoShoot;
import org.firstinspires.ftc.teamcode.mechwarriors.hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.mechwarriors.hardware.LEDIndicator;
import org.firstinspires.ftc.teamcode.mechwarriors.opmodes.testopmodes.IndicatorLight;
import org.firstinspires.ftc.teamcode.mechwarriors.opmodes.testopmodes.IndicatorLightColor;

import java.util.List;

@Config
@TeleOp
public class DecodeTeleOpMode extends OpMode {
    private ColorSensor colorSensor;
    private DistanceSensor distanceSensor;

    private Servo light;

    LEDIndicator leftLedIndicatorLight;
    LEDIndicator rightLedIndicatorLight;


    PIDController autoAimPID;
    public static double AUTO_AIM_KP = 0.05;
    public static double AUTO_AIM_KI = 0;
    public static double AUTO_AIM_KD = 0.0007;

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
    AutoShoot autoShoot;

    Boolean isAutoShootRunning = false;

    IndicatorLight indicatorLight;

    @Override
    public void init() {

        leftLedIndicatorLight = new LEDIndicator(hardwareMap, "leftDirectionIndicator");
        rightLedIndicatorLight = new LEDIndicator(hardwareMap, "rightDirectionIndicator");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        drivetrain = new DriveTrain(hardwareMap, telemetry);

        // Artifact Intake
        artifactIntaker = new ArtifactIntaker(hardwareMap);

        // Artifact Sorter
        artifactSorter = new ArtifactSorter(hardwareMap, telemetry);

        // ArtifactSorterMode mode = (ArtifactSorterMode) blackboard.getOrDefault(BlackboardItems.SORTER_MODE.name(), ArtifactSorterMode.LAUNCH);
        //  artifactSorter.setArtifactSorterMode(mode);


        sorterTouchSensor = hardwareMap.get(DigitalChannel.class, "sorterTouchSensor");

        artifactLauncher = new ArtifactLauncher(hardwareMap, telemetry);

        autoShoot = new AutoShoot(artifactLauncher, artifactSorter, telemetry);

        autoAimPID = new PIDController(AUTO_AIM_KP, AUTO_AIM_KI, AUTO_AIM_KD);

        indicatorLight = new IndicatorLight(hardwareMap.get(Servo.class, "indicator_light"));
    }

    @Override
    public void start() {
        limelight.start();

        artifactSorter.init();
    }


    @Override
    public void loop() {
        artifactLauncher.checkSpeed();


        double distance;
        boolean isRunning;

        double limelightTargetX = 0;

        LLResult result = limelight.getLatestResult();
        List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
        if (!fiducialResults.isEmpty()) {
            telemetry.addData("result.getTx()", result.getTx());
            limelightTargetX = -result.getTx();
            LLResultTypes.FiducialResult aprilTagResult = fiducialResults.get(0);
            int id = aprilTagResult.getFiducialId();
            telemetry.addData("Seeing April Tag", id);
            indicatorLight.setColor(IndicatorLightColor.GREEN);
            // limelightTargetX = aprilTagResult.getTargetXDegrees();
//            if (limelightTargetX > 1.0) {
//                // LEFT
//                rightLedIndicatorLight.setColor(LEDIndicator.LEDColor.RED);
//                leftLedIndicatorLight.setColor(LEDIndicator.LEDColor.GREEN);
//            } else if (limelightTargetX < -1.0) {
//                // RIGHT
//                rightLedIndicatorLight.setColor(LEDIndicator.LEDColor.GREEN);
//                leftLedIndicatorLight.setColor(LEDIndicator.LEDColor.RED);
//            } else if (limelightTargetX <= 1.0 && limelightTargetX >= -1.0) {
//                // CENTER
//                rightLedIndicatorLight.setColor(LEDIndicator.LEDColor.GREEN);
//                leftLedIndicatorLight.setColor(LEDIndicator.LEDColor.GREEN);
//            }
        } else {
            indicatorLight.setColor(IndicatorLightColor.RED);
            rightLedIndicatorLight.setColor(LEDIndicator.LEDColor.OFF);
            leftLedIndicatorLight.setColor(LEDIndicator.LEDColor.OFF);
        }

        telemetry.addData("BallColor", determineColor());

        double y = gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;

        double rx;
        if (gamepad1.right_trigger > 0.5) {
            autoAimPID.setPID(AUTO_AIM_KP, AUTO_AIM_KI, AUTO_AIM_KD);
            double calculatedX = autoAimPID.calculate(limelightTargetX, 0);
            telemetry.addData("calculatedX", calculatedX);
            rx = calculatedX;
        } else {
            rx = gamepad1.right_stick_x;
        }
        telemetry.addData("rx", rx);
        telemetry.addData("calculated rx", autoAimPID.calculate(limelightTargetX));

        drivetrain.drive(x, y, rx);

        if (gamepad1.left_bumper) {
            drivetrain.setSlowMode(true);
        } else if (gamepad1.right_bumper) {
            drivetrain.setSlowMode(false);
        }


        telemetry.addData("gamepad2.right_stick_y", gamepad2.right_stick_y);

        telemetry.addData("launcherArmPosition", launcherArmPosition);
        if (gamepad2.shareWasReleased()) {
            artifactLauncher.setShootingMode();
        }


        //SuperSlowMode
        if (gamepad1.x) {
            drivetrain.setSuperSlowMode(true);
        } else if (gamepad1.y) {
            drivetrain.setSuperSlowMode(false);
        }

        // Intake motor
        if (gamepad2.b &&
                artifactIntaker.getSweeperPosition() == ArtifactIntaker.SweeperPosition.FORWARD &&
                artifactSorter.getArtifactSorterMode() == ArtifactSorterMode.INTAKE) {
            artifactIntaker.runIntakeMotor();
        } else if (gamepad2.x &&
                artifactIntaker.getSweeperPosition() == ArtifactIntaker.SweeperPosition.FORWARD &&
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
        telemetry.addData("launchServoPosition", artifactLauncher.getLaunchServoPosition());
        //AutoShoot
        if (gamepad2.dpadRightWasReleased() && artifactSorter.getArtifactSorterMode() == ArtifactSorterMode.LAUNCH) {
            autoShoot.startAutoShoot();
            isAutoShootRunning = true;
        } else if (gamepad2.dpadLeftWasReleased()) {
            isAutoShootRunning = false;
        }
        String autoShootState = autoShoot.autoShoot(isAutoShootRunning);
        if (isAutoShootRunning != null && !isAutoShootRunning) {
            isAutoShootRunning = null;
        }

        if (autoShootState.isEmpty()) {
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
        return null;
    }

    private double getDistanceFromTag(double y) {
        double a = 5208.601;
        double b = -2.008664;
        return Math.pow(y / a, 1.0 / b);
    }


}