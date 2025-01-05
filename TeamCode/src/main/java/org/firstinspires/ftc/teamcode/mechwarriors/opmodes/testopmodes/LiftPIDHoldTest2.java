package org.firstinspires.ftc.teamcode.mechwarriors.opmodes.testopmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PController;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config
@Disabled
@TeleOp(group = "Testing", name = "LiftPIDHoldTest2")
public class LiftPIDHoldTest2 extends OpMode {

    DcMotorEx clawArmMotor;
    DcMotorEx liftMotor;

    PController liftMotorPIDController;
    public static double liftKp = 0.01;
    double liftTargetPosition = 0;

    PController clawArmPIDController;
    public static double clawArmKp = 0.02;
    double clawArmTargetPosition = 0;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        clawArmMotor = hardwareMap.get(DcMotorEx.class, "armLiftMotor");
        clawArmMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        clawArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        clawArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        clawArmPIDController = new PController(clawArmKp);

        liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotorPIDController = new PController(liftKp);
    }

    @Override
    public void loop() {


        double liftMotorCurrentPosition = liftMotor.getCurrentPosition();
        double liftPower = liftMotorPIDController.calculate(liftMotorCurrentPosition, liftTargetPosition);
        liftMotor.setPower(liftPower);

        telemetry.addData("Lift Target position", liftTargetPosition);
        telemetry.addData("Lift Current position", liftMotorCurrentPosition);
        telemetry.addData("Lift Power", liftPower);
        telemetry.addData("Lift Motor current", liftMotor.getCurrent(CurrentUnit.AMPS));

        if (gamepad1.dpad_up) {
            liftTargetPosition = 4000;
        } else if (gamepad1.dpad_left) {
            liftTargetPosition = 400;
        } else if (gamepad1.dpad_down) {
            if (clawArmTargetPosition < 200) {
                liftTargetPosition = 0;
            }
        }

        double clawArmMotorCurrentPosition = clawArmMotor.getCurrentPosition();
        clawArmPIDController.setP(clawArmKp);
        double clawArmPower = clawArmPIDController.calculate(clawArmMotorCurrentPosition, clawArmTargetPosition);
        clawArmPower = Math.max(-0.40, Math.min(0.25, clawArmPower));
        clawArmMotor.setPower(clawArmPower);

        telemetry.addLine("");
        telemetry.addData("Claw Arm Target position", clawArmTargetPosition);
        telemetry.addData("Claw Arm Current position", clawArmMotorCurrentPosition);
        telemetry.addData("Claw Arm Power", clawArmPower);
        telemetry.addData("Claw Arm Motor current", clawArmMotor.getCurrent(CurrentUnit.AMPS));

        if (gamepad1.y) {
            clawArmTargetPosition = 0;
        } else if (gamepad1.x) {
            if (liftTargetPosition > 300) {
                clawArmTargetPosition = 100;
            }
        } else if (gamepad1.a) {
            if (liftTargetPosition > 300) {
                clawArmTargetPosition = 350;
            }
        } else if (gamepad1.b) {
            if (liftTargetPosition > 300) {
                clawArmTargetPosition = 495;
            }
        }


        telemetry.update();
    }


    @Override
    public void stop() {
        liftMotor.setPower(0);
    }
}
