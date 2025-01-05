package org.firstinspires.ftc.teamcode.mechwarriors.opmodes.old;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.mechwarriors.Utilities;
import org.firstinspires.ftc.teamcode.mechwarriors.behaviors.Behavior;
import org.firstinspires.ftc.teamcode.mechwarriors.behaviors.CloseClaw;
import org.firstinspires.ftc.teamcode.mechwarriors.behaviors.OpenClaw;
import org.firstinspires.ftc.teamcode.mechwarriors.hardware.Claw;
import org.firstinspires.ftc.teamcode.mechwarriors.hardware.IntoTheDeepRobot;

import java.util.ArrayList;
import java.util.List;

@TeleOp(group = "CenterStage")
@Disabled
public class CenterStageOpMode extends OpMode {
    IntoTheDeepRobot robot;
    Claw leftClaw;
    Claw rightClaw;
    boolean slowMode = false;
    double zeroPitchAngle;

    DcMotor robotLift;
    Servo robotLiftServoArm;

    boolean pixelBeingLifted = false;
    int pixelBeingLiftedState = 0;
    List<Behavior> pickUpPixelBehaviors = new ArrayList<>();

    DistanceSensor leftDistanceSensor;
    DistanceSensor rightDistanceSensor;

    @Override
    public void init() {
        robot = new IntoTheDeepRobot(hardwareMap);
        leftClaw = robot.getSampleClaw();
        zeroPitchAngle = robot.getPitchAngle();
        robotLift = hardwareMap.get(DcMotor.class, "robotLift");
        robotLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robotLiftServoArm = hardwareMap.get(Servo.class, "robotLiftServoArm");
        robotLiftServoArm.setPosition(0.90);
        robotLift.setDirection(DcMotorSimple.Direction.REVERSE);


        leftDistanceSensor = hardwareMap.get(DistanceSensor.class, "leftDistanceSensor");
        rightDistanceSensor = hardwareMap.get(DistanceSensor.class, "rightDistanceSensor");

        leftClaw.open();
        rightClaw.open();


        pickUpPixelBehaviors.add(new CloseClaw(telemetry, leftClaw));
        pickUpPixelBehaviors.add(new CloseClaw(telemetry, rightClaw));
        pickUpPixelBehaviors.add(new OpenClaw(telemetry, leftClaw));
        pickUpPixelBehaviors.add(new OpenClaw(telemetry, rightClaw));
    }

    @Override
    public void loop() {
        telemetry.addData("motors", robot.getDriveTicksString());

        telemetry.addData("leftDistanceSensor", leftDistanceSensor.getDistance(DistanceUnit.INCH));
        telemetry.addData("rightDistanceSensor", rightDistanceSensor.getDistance(DistanceUnit.INCH));
        // Gamepad 1

        if (gamepad1.left_bumper) {
            slowMode = true;
        }
        if (gamepad1.right_bumper) {
            slowMode = false;
        }

        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x * 1.1;
        double rx = gamepad1.right_stick_x;

        y = Utilities.squareInputWithSign(y);
        x = Utilities.squareInputWithSign(x);
        rx = Utilities.squareInputWithSign(rx);

        if (slowMode) {
            x = x * 0.4;
            y = y * 0.4;
            rx = rx * 0.4;
        }

        robot.drive(x, y, rx);
        telemetry.addData("x", x);
        telemetry.addData("y", y);
        telemetry.addData("rx", rx);

        telemetry.addData("Lift ticks: ", robot.getLift().getLiftTicks());

        if (gamepad1.y) {
            robotLiftServoArm.setPosition(0);
        } else if (gamepad1.a) {
            robotLiftServoArm.setPosition(1.0);
        }

        if (gamepad1.dpad_up) {
            robotLift.setPower(1.0);
        } else if (gamepad1.dpad_down) {
            robotLift.setPower(-1.0);
        } else {
            robotLift.setPower(0);
        }

        if (gamepad1.b) {
           // planeLauncher.launch();
        }

        // Gamepad 2
        if (gamepad2.x) {
            pixelBeingLifted = true;
        } else if (gamepad2.b) {
            pixelBeingLifted = false;
            pixelBeingLiftedState = 0;
        }

        if (pixelBeingLifted) {
            if (pixelBeingLiftedState < pickUpPixelBehaviors.size()) {
                //checking to see if the current behavior is not done, run that behavior
                if (!pickUpPixelBehaviors.get(pixelBeingLiftedState).isDone()) {
                    telemetry.addData("Running behavior", pickUpPixelBehaviors.get(pixelBeingLiftedState).getName());
                    pickUpPixelBehaviors.get(pixelBeingLiftedState).run();
                } else {
                    //increments the behavior
                    pixelBeingLiftedState++;
                    //starts next behavior if there are any left
                    if (pixelBeingLiftedState < pickUpPixelBehaviors.size()) {
                        pickUpPixelBehaviors.get(pixelBeingLiftedState).start();
                    }
                }
            } else {
                //if all behaviors are finished, stop the robot
                telemetry.addLine("Pixels lift done");
                pixelBeingLifted = false;
                pixelBeingLiftedState = 0;
            }
        } else {
            if (gamepad2.dpad_up) {
                robot.getLift().liftArmUp();
                telemetry.addData("Lift", "Up");
            } else if (gamepad2.dpad_down) {
                robot.getLift().liftArmDown();
                telemetry.addData("Lift", "Down");
            } else {
                robot.getLift().liftArmStop();
                telemetry.addData("Lift", "Stop");
            }


            rightClaw.toggleOpen(gamepad2.right_bumper, telemetry);
            leftClaw.toggleOpen(gamepad2.left_bumper, telemetry);

        }
    }
}
