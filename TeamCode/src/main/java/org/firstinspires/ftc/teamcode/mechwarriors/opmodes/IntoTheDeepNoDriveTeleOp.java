package org.firstinspires.ftc.teamcode.mechwarriors.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(group = "IntoTheDeep", name = "No Drive TeleOp")
@Disabled
public class IntoTheDeepNoDriveTeleOp extends OpMode {

    DcMotorEx clawArmMotor;
    DcMotorEx liftMotor;
    DcMotorEx rightHangMotor;
    DcMotorEx leftHangMotor;
    Servo clawServo;

    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backRightMotor;

    @Override
    public void init() {
        rightHangMotor = hardwareMap.get(DcMotorEx.class, "rightHangMotor");
        leftHangMotor = hardwareMap.get(DcMotorEx.class, "leftHangMotor");
        rightHangMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftHangMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        clawServo = hardwareMap.get(Servo.class, "clawServo");
        clawArmMotor = hardwareMap.get(DcMotorEx.class, "armLiftMotor");
        clawArmMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");
        rightHangMotor = hardwareMap.get(DcMotorEx.class, "rightHangMotor");
        leftHangMotor = hardwareMap.get(DcMotorEx.class, "leftHangMotor");
        rightHangMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftHangMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // leftHangMotor
        // rightHangMotor
        // rightHangServo
    }

    @Override
    public void loop() {

        if (gamepad1.dpad_up) {
            leftHangMotor.setPower(1);
        } else if (gamepad1.dpad_down) {
            leftHangMotor.setPower(-1);
        } else {
            leftHangMotor.setPower(0);
        }

        if (gamepad1.right_bumper) {
            rightHangMotor.setPower(1);
        } else if (gamepad1.left_bumper) {
            rightHangMotor.setPower(-1);
        } else {
            rightHangMotor.setPower(0);
        }

//        if (gamepad1.dpad_up) {
//            clawArmMotor.setPower(0.4);
//        } else if (gamepad1.dpad_down) {
//            clawArmMotor.setPower(-0.25);
//        } else {
//            clawArmMotor.setPower(0);
//        }
        telemetry.addData("ClawArmMotor Position", clawArmMotor.getCurrentPosition());

        /////////////////////////////////////////
        if (gamepad1.a) {
            liftMotor.setPower(1);
        } else if (gamepad1.b) {
            liftMotor.setPower(-1);
        } else {
            liftMotor.setPower(0);
        }
        telemetry.addData("LiftMotor " +
                "Position", clawArmMotor.getCurrentPosition());

        /////////////////////////////////////////
        if (gamepad1.x) {
            clawServo.setPosition(0.5);
        }
        if (gamepad1.y) {
            clawServo.setPosition(0.1);
        }



    }
}
