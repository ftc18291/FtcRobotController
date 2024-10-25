package org.firstinspires.ftc.teamcode.mechwarriors.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class DeepDiveTeleOp2 extends OpMode {

    DcMotorEx clawArmMotor;
    DcMotorEx liftMotor;
    Servo clawServo;

    @Override
    public void init() {

        clawServo = hardwareMap.get(Servo.class, "clawServo");
        clawArmMotor = hardwareMap.get(DcMotorEx.class, "armLiftMotor");
        clawArmMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");
    }

    @Override
    public void loop() {

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

        /////////////////////////////////////////
        if (gamepad1.x) {
            clawServo.setPosition(0.5);
        }
        if (gamepad1.y) {
            clawServo.setPosition(0.1);

        }

    }
}
