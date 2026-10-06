package org.firstinspires.ftc.teamcode.mechwarriors.opmodes;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp
public class testGamepads extends OpMode {
Gamepad gamepad;

    @Override
    public void init() {
gamepad = gamepad2;
    }

    @Override
    public void loop() {

        if (gamepad.a) {
            telemetry.addLine("aButtonPressed1 xx");
        }
        if (gamepad.b) {
            telemetry.addLine("bButtonPressed1");
        }
        if (gamepad.x) {
            telemetry.addLine("xButtonPressed1");
        }
        if (gamepad.y) {
            telemetry.addLine("yButtonPressed1");
        }
        if (gamepad.left_bumper) {
            telemetry.addLine("leftBumperButtonPressed1");
        }
        if (gamepad.right_bumper) {
            telemetry.addLine("rightBumperButtonPressed1");
        }
        if (gamepad.left_trigger > .1) {
            telemetry.addLine("leftTriggerButtonPressed1");
        }
        if (gamepad.right_trigger > .1) {
            telemetry.addLine("rightTriggerButtonPressed1");
        }
        if (gamepad.dpad_up) {
            telemetry.addLine("dpadUpButtonPressed1");
        }
        if (gamepad.dpad_down) {
            telemetry.addLine("dpadDownButtonPressed1");
        }
        if (gamepad.dpad_left) {
            telemetry.addLine("dpadLeftButtonPressed1");
        }
        if (gamepad.dpad_right) {
            telemetry.addLine("dpadRightButtonPressed1");
        }
        telemetry.addData("LeftStickxPushed1", gamepad1.left_stick_x);
        telemetry.addData("leftStickyPushed1", gamepad1.left_stick_y);
        telemetry.addData("rightStickxPushed1", gamepad1.right_stick_x);
        telemetry.addData("rightStickyPushed1", gamepad1.right_stick_y);

        telemetry.addData("LeftStickxPushed1", gamepad2.left_stick_x);
        telemetry.addData("leftStickyPushed1", gamepad2.left_stick_y);
        telemetry.addData("rightStickxPushed1", gamepad2.right_stick_x);
        telemetry.addData("rightStickyPushed1", gamepad2.right_stick_y);


        if (gamepad.left_stick_button) {
            telemetry.addLine("leftStickButtonPushed1");
        }
        if (gamepad.right_stick_button) {
            telemetry.addLine("rightStickButtonPressed1");
        }

    }
}
