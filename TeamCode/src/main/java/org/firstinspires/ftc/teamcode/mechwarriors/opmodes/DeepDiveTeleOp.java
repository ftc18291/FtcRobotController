package org.firstinspires.ftc.teamcode.mechwarriors.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class DeepDiveTeleOp extends OpMode {

    DcMotorEx clawArmMotor;
    DcMotorEx liftMotor;

    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backRightMotor;


    @Override
    public void init() {
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor"); // Expansion Hub Port 0
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");// Expansion Hub Port 1
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");// Expansion Hub Port 2
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");// Expansion Hub Port 3

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        clawArmMotor = hardwareMap.get(DcMotorEx.class, "armLiftMotor");
        clawArmMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");
    }

    @Override
    public void loop() {
        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);

        ////////////////////////////////////////////
        if (gamepad1.dpad_up) {
            clawArmMotor.setPower(0.4);
        } else if (gamepad1.dpad_down) {
            clawArmMotor.setPower(-0.25);
        } else {
            clawArmMotor.setPower(0);
        }
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
    }
}
