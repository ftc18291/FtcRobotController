package org.firstinspires.ftc.teamcode.mechwarriors.opmodes;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.mechwarriors.hardware.ArtifactIntaker;
import org.firstinspires.ftc.teamcode.mechwarriors.hardware.ArtifactSorter;
import org.firstinspires.ftc.teamcode.mechwarriors.hardware.ArtifactSorterMode;
import org.firstinspires.ftc.teamcode.mechwarriors.hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.mechwarriors.opmodes.testopmodes.IndicatorLight;
import org.firstinspires.ftc.teamcode.mechwarriors.opmodes.testopmodes.IndicatorLightColor;

@Config
@TeleOp
public class DecodeTeleOpMode extends OpMode {
    private ColorSensor colorSensor;
    private DistanceSensor distanceSensor;

    private Servo light;

    IndicatorLight indicatorLight;

    //Limelight3A limelight;

    private DriveTrain drivetrain;

    DcMotorEx launcherMotor;
    Servo launcherServo;

    DcMotorEx sorterMotor;

    Boolean sorterRotateButtonPressed = false;

    Boolean sorterSetIntakeModeButtonPressed = false;
    Boolean sorterSetLaunchModeButtonPressed = false;


    DigitalChannel sorterTouchSensor;

    ArtifactIntaker artifactIntaker;
    ArtifactSorter artifactSorter;

    @Override
    public void init() {
        indicatorLight = new IndicatorLight(hardwareMap.get(Servo.class, "indicator_light"));

        colorSensor = hardwareMap.get(ColorSensor.class, "artifactColorSensor");

        //light = hardwareMap.get(Servo.class, "light");

        //limelight = hardwareMap.get(Limelight3A.class, "limelight");
        //limelight.pipelineSwitch(0);
        drivetrain = new DriveTrain(hardwareMap, telemetry);

        // Artifact Intake
        artifactIntaker = new ArtifactIntaker(hardwareMap);

        // Artifact Sorter
        artifactSorter = new ArtifactSorter(hardwareMap);

        sorterTouchSensor = hardwareMap.get(DigitalChannel.class, "sorterTouchSensor");

        // Artifact Launcher
        launcherMotor = hardwareMap.get(DcMotorEx.class, "launcherMotor");
        launcherMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        launcherServo = hardwareMap.get(Servo.class, "launcherServo");
        launcherServo.scaleRange(0.65, 1.0);
        launcherServo.setPosition(0);
    }

    @Override
    public void start() {
        //limelight.start();

        artifactSorter.init();
    }


    @Override
    public void loop() {
        double distance;


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


        String color = determineColor();

        switch (color) {
            case "Green":
                indicatorLight.setColor(IndicatorLightColor.GREEN);
                break;
            case "Purple":
                indicatorLight.setColor(IndicatorLightColor.PURPLE);
                break;
        }

        double y = gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;
        drivetrain.drive(x, y, rx);

        if (gamepad1.left_bumper) {
            drivetrain.setSlowMode(true);
        } else if (gamepad1.right_bumper) {
            drivetrain.setSlowMode(false);
        }

        if (gamepad1.x) {
            drivetrain.setSuperSlowMode(true);
        } else if (gamepad1.y) {
            drivetrain.setSuperSlowMode(false);
        }

        // Intake motor
        if (gamepad2.b &&
                artifactIntaker.getSweeperPosition() == 0 &&
                artifactSorter.getArtifactSorterMode() == ArtifactSorterMode.INTAKE) {
            artifactIntaker.runIntakeMotor();
        } else if (gamepad2.x &&
                artifactSorter.getArtifactSorterMode() == ArtifactSorterMode.INTAKE) {
            artifactIntaker.reverseIntakeMotor();
        } else {
            artifactIntaker.stopIntakeMotor();
        }

        // Intake sweepers
        if (gamepad2.dpad_down) {
            artifactIntaker.setSweeperToRearPosition();
        }
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
        if (gamepad2.right_trigger > 0.5) {
            launcherMotor.setVelocity(2200);
        } else {
            launcherMotor.setVelocity(0);
        }
        telemetry.addData("launcherMotor velocity", launcherMotor.getVelocity());

        if (gamepad2.a && artifactSorter.getArtifactSorterMode() == ArtifactSorterMode.LAUNCH) {
            launcherServo.setPosition(1.0);
        } else {
            launcherServo.setPosition(0);
        }

        // Alternate launch
        //        if (gamepad2.b) {
        //            launcherMotor.setVelocity(2000);
        //            if (launcherMotor.getVelocity() > 1990) {
        //                launcherServo.setPosition(1.0);
        //            }
        //        } else {
        //            launcherMotor.setVelocity(0);
        //            launcherServo.setPosition(0);
        //        }
    }

    private boolean getSorterSensorState() {
        return !sorterTouchSensor.getState();
    }

    public String determineColor() {
        int green = colorSensor.green();
        int red = colorSensor.red();
        if (red > 100 && green > 150) {
            return "Purple";
        } else {
            return "Green";
        }


    }

    private double getDistanceFromTag(double y) {
        double a = 5208.601;
        double b = -2.008664;
        return Math.pow(y / a, 1.0 / b);
    }
}