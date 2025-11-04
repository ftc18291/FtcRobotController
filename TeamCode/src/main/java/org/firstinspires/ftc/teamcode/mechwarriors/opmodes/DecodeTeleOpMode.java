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

import org.firstinspires.ftc.teamcode.mechwarriors.hardware.ArtifactSorter;
import org.firstinspires.ftc.teamcode.mechwarriors.hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.mechwarriors.opmodes.testopmodes.IndicatorLight;

@Config
@TeleOp
public class DecodeTeleOpMode extends OpMode {
    private ColorSensor colorSensor;
    private DistanceSensor distanceSensor;

    private Servo light;

    IndicatorLight indicatorLight;

    Limelight3A limelight;

    private DriveTrain drivetrain;

    DcMotorEx launcherMotor;
    Servo launcherServo;

    DcMotorEx sorterMotor;

    Boolean sorterRotateButtonPressed = false;

    Boolean sorterSetIntakeModeButtonPressed = false;
    Boolean sorterSetLaunchModeButtonPressed = false;


    DigitalChannel sorterTouchSensor;

    ArtifactSorter artifactSorter;

   // private static final double TICKS_PER_ROTATION = 587.04;
  //  private static final double TICKS_PER_SLOT = TICKS_PER_ROTATION / 3;
  //  private static final double MOVE_TO_INTAKE_POSITION_TICKS = 55;
  //  private static final double MOVE_TO_LAUNCH_POSITION_TICKS = TICKS_PER_SLOT - MOVE_TO_INTAKE_POSITION_TICKS;

    //private double currentTicksTarget = 0;

   // private ArtifactSorterMode artifactSorterMode = ArtifactSorterMode.LAUNCH;

   // public static double MOTOR_SPEED = 0.3;
  //  public static double NEW_P = 20;
   // public static double NEW_I = 10;
   // public static double NEW_D = 0.1;

    DcMotorEx intakeMotor;

    Servo leftSweeperServo;
    Servo rightSweeperServo;

    @Override
    public void init() {
        //indicatorLight = new IndicatorLight(hardwareMap.get(Servo.class, "light"));

        //colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color_distance");
        // distanceSensor = hardwareMap.get(DistanceSensor.class, "sensor_color_distance");

        //light = hardwareMap.get(Servo.class, "light");

        //limelight = hardwareMap.get(Limelight3A.class, "limelight");
        //limelight.pipelineSwitch(0);
        drivetrain = new DriveTrain(hardwareMap, telemetry);

        // Artifact Intake
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftSweeperServo = hardwareMap.get(Servo.class, "leftSweeperServo");
        rightSweeperServo = hardwareMap.get(Servo.class, "rightSweeperServo");
        leftSweeperServo.setDirection(Servo.Direction.REVERSE);
        leftSweeperServo.scaleRange(0.1, 0.92); // reverse/forward
        rightSweeperServo.scaleRange(0.05, 0.9); //

        leftSweeperServo.setPosition(0);
        rightSweeperServo.setPosition(0);

        // Artifact Sorter
        artifactSorter = new ArtifactSorter(hardwareMap);

//        sorterMotor = hardwareMap.get(DcMotorEx.class, "sorterMotor");
//        sorterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        sorterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        PIDFCoefficients pidNew = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, 0);
//        sorterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidNew);
//        sorterMotor.setTargetPosition(0);
//        sorterMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
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

        //sorterMotor.setPower(MOTOR_SPEED);
        //sorterMotor.setTargetPosition((int) currentTicksTarget);
    }


    @Override
    public void loop() {
        double distance;


//        telemetry.addData("Red", colorSensor.red());
//        telemetry.addData("Blue", colorSensor.blue());
//        telemetry.addData("Green", colorSensor.green());
//        telemetry.addData("ARGB", colorSensor.argb());
//        telemetry.addData("Distance", distanceSensor.getDistance(DistanceUnit.CM));
//         telemetry.addData("BallColor", determineColor());
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
//                light.setPosition(.500);
//                break;
//            case "Purple":
//                light.setPosition(.722);
//                break;
//
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

        if (gamepad1.x) {
            drivetrain.setSuperSlowMode(true);
        } else if (gamepad1.y) {
            drivetrain.setSuperSlowMode(false);
        }

        if (gamepad2.right_trigger > 0.5) {
            launcherMotor.setVelocity(2400);
        } else {
            launcherMotor.setVelocity(0);
        }

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

        telemetry.addData("sorterTouchSensor", getSorterSensorState());
        telemetry.addData("artifactSorterMode", artifactSorter.getArtifactSorterMode());
       // telemetry.addData("sorterMotor ticks", sorterMotor.getCurrentPosition());
        telemetry.addData("currentTicksTarget", artifactSorter.getCurrentTicksTarget());

        if (gamepad2.left_trigger > 0.5) {
            sorterRotateButtonPressed = true;
        }
        if (gamepad2.left_trigger < 0.5 && sorterRotateButtonPressed) {
            sorterRotateButtonPressed = false;
            //currentTicksTarget = currentTicksTarget + TICKS_PER_SLOT;
            artifactSorter.rotateOneSlot();
        }

        // Rotate from launch to intake position
        if (gamepad2.left_bumper) {
            sorterSetIntakeModeButtonPressed = true;
        } else if (!gamepad2.left_bumper && sorterSetIntakeModeButtonPressed) {
            sorterSetIntakeModeButtonPressed = false;
            artifactSorter.setArtifactSorterMode(ArtifactSorterMode.INTAKE);
//            if (artifactSorterMode == ArtifactSorterMode.LAUNCH) {
//                artifactSorterMode = ArtifactSorterMode.INTAKE;
//                currentTicksTarget = currentTicksTarget + MOVE_TO_INTAKE_POSITION_TICKS;
//            } else {
//                telemetry.addLine("Already in intake position");
//            }
        }
        // Rotate from intake to launch position
        if (gamepad2.right_bumper) {
            sorterSetLaunchModeButtonPressed = true;
        } else if (!gamepad2.right_bumper && sorterSetLaunchModeButtonPressed) {
            sorterSetLaunchModeButtonPressed = false;
            artifactSorter.setArtifactSorterMode(ArtifactSorterMode.LAUNCH);
//            if (artifactSorterMode == ArtifactSorterMode.INTAKE) {
//                artifactSorterMode = ArtifactSorterMode.LAUNCH;
//                currentTicksTarget = currentTicksTarget + MOVE_TO_LAUNCH_POSITION_TICKS;
//            } else {
//                telemetry.addLine("Already in launch position");
//            }
        }

       // sorterMotor.setTargetPosition((int) currentTicksTarget);


        if (gamepad2.b && leftSweeperServo.getPosition() == 0 && artifactSorter.getArtifactSorterMode() == ArtifactSorterMode.INTAKE) {
            intakeMotor.setVelocity(1000);
        } else {
            intakeMotor.setVelocity(0.0);
        }

        if (gamepad2.dpad_down) {
            leftSweeperServo.setPosition(1.0);
            rightSweeperServo.setPosition(1.0);
        }
        if (gamepad2.dpad_up) {
            leftSweeperServo.setPosition(0);
            rightSweeperServo.setPosition(0);
        }

        telemetry.addData("launcherMotor velocity", launcherMotor.getVelocity());

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