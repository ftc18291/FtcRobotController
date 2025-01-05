package org.firstinspires.ftc.teamcode.mechwarriors.opmodes.testopmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ElevatorFeedforward;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
@Config
@Disabled
@TeleOp(group = "Testing", name = "LiftPIDHoldTest")
public class LiftPIDHoldTest extends OpMode {

    DcMotorEx clawArmMotor;
    DcMotorEx liftMotor;

    public static double liftKp = 0.005;
    public static double liftKi = 0.0;
    public static double liftKd = 0.0;

    double liftTargetPosition = 0;
    double liftPreviousError = 0;
    double liftIntegral = 0;

    double clawArmKp = 0.02;
    double clawArmKi = 0.0;
    double clawArmKd = 0.0;

    double clawArmTargetPosition = 0;
    double clawArmPreviousError = 0;
    double clawArmIntegral = 0;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        clawArmMotor = hardwareMap.get(DcMotorEx.class, "armLiftMotor");
        clawArmMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        clawArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        clawArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        PIDController liftMotorPIDController = new PIDController(liftKp, liftKi, liftKd);
    }

    @Override
    public void loop() {

        {
            double currentPosition = liftMotor.getCurrentPosition();

            double error = liftTargetPosition - currentPosition;
            liftIntegral += error;
            double derivative = error - liftPreviousError;
            double power = (liftKp * error) + (liftKi * liftIntegral) + (liftKd * derivative);
            power = Math.max(-1.0, Math.min(1.0, power));
            liftMotor.setPower(power);
            liftPreviousError = error;

            telemetry.addData("Lift Target position", liftTargetPosition);
            telemetry.addData("Lift Current position", currentPosition);
            telemetry.addData("Lift error", error);
            telemetry.addData("Lift Power", power);
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
        }

        {
            double currentPosition = clawArmMotor.getCurrentPosition();
            double error = clawArmTargetPosition - currentPosition;
            clawArmIntegral += error;
            double derivative = error - clawArmPreviousError;
            double power = (clawArmKp * error) + (clawArmKi * clawArmIntegral) + (clawArmKd * derivative);
            power = Math.max(-0.40, Math.min(0.25, power));
            clawArmMotor.setPower(power);
            clawArmPreviousError = error;

            telemetry.addLine("");
            telemetry.addData("Claw Arm Target position", clawArmTargetPosition);
            telemetry.addData("Claw Arm Current position", currentPosition);
            telemetry.addData("Claw Arm error", error);
            telemetry.addData("Claw Arm Power", power);
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
        }

        telemetry.update();
    }


    @Override
    public void stop() {
        liftMotor.setPower(0);
    }
}
