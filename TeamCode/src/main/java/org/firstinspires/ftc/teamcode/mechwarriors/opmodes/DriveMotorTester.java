package org.firstinspires.ftc.teamcode.mechwarriors.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class DriveMotorTester extends OpMode {

    DcMotorEx leftFrontMotor;
    DcMotorEx rightFrontMotor;
    DcMotorEx leftRearMotor;
    DcMotorEx rightRearMotor;

    @Override
    public void init() {
        leftFrontMotor = hardwareMap.get(DcMotorEx.class, "leftFrontMotor");
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        rightFrontMotor = hardwareMap.get(DcMotorEx.class, "rightFrontMotor");
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftRearMotor = hardwareMap.get(DcMotorEx.class, "leftRearMotor");
        leftRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRearMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        rightRearMotor = hardwareMap.get(DcMotorEx.class, "rightRearMotor");
        rightRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {

        if (gamepad1.a) {
            leftFrontMotor.setPower(1.0);
            telemetry.addLine("A pressed, running left front motor");
        } else {
            leftFrontMotor.setPower(0);
            telemetry.addLine("A to run left front motor");
        }
        if (gamepad1.b) {
            rightFrontMotor.setPower(1.0);
            telemetry.addLine("B pressed, running right front motor");
        } else {
            rightFrontMotor.setPower(0);
            telemetry.addLine("B to run right front motor");
        }
        if (gamepad1.x) {
            leftRearMotor.setPower(1.0);
            telemetry.addLine("X pressed, running left rear motor");
        } else  {
            leftRearMotor.setPower(0);
            telemetry.addLine("X to run left rear motor");
        }
        if (gamepad1.y) {
            rightRearMotor.setPower(1.0);
            telemetry.addLine("Y pressed, running right rear motor");
        } else {
            rightRearMotor.setPower(0);
            telemetry.addLine("Y to run right rear motor");
        }

        telemetry.addLine(" ");
        telemetry.addData("leftFrontMotor", leftFrontMotor.getPower());
        telemetry.addData("rightFrontMotor", rightFrontMotor.getPower());
        telemetry.addData("leftRearMotor", leftRearMotor.getPower());
        telemetry.addData("rightRearMotor", rightRearMotor.getPower());
    }
}
