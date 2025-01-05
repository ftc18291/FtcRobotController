package org.firstinspires.ftc.teamcode.mechwarriors.opmodes.testopmodes;

import com.arcrobotics.ftclib.controller.wpilibcontroller.ElevatorFeedforward;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Disabled
@TeleOp(group = "Testing", name = "ElevatorFeedForwardText")
public class ElevatorFeedForwardText extends OpMode {

    DcMotorEx clawArmMotor;
    DcMotorEx liftMotor;
    private ElevatorFeedforward elevatorFeedforward;

    private static final double kS = 0.3;
    private static final double kG = 0.05;
    private static final double kV = 0.8;
    private static final double kA = 0.0;


    private static final double MAX_VELOCITY = 2000.0;
    private static final double MAX_ACCELERATION = 3000.0;

    double liftKp = 0.005;
    double liftKi = 0.0;
    double liftKd = 0.0;

    double liftTargetPosition = 0;
    double liftPreviousError = 0;
    double liftIntegral = 0;

    double clawArmKp = 0.02;
    double clawArmKi = 0.0;
    double clawArmKd = 0.0;

    double clawArmTargetPosition = 0;
    double clawArmPreviousError = 0;
    double clawArmIntegral = 0;

    ElevatorFeedforward feedforward;

    @Override
    public void init() {
//        clawArmMotor = hardwareMap.get(DcMotorEx.class, "armLiftMotor");
//        clawArmMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        clawArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        clawArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        elevatorFeedforward = new ElevatorFeedforward(kS, kG, kV, kA);
    }

    @Override
    public void loop() {

        {
            double currentPosition = liftMotor.getCurrentPosition();
//            double error = liftTargetPosition - currentPosition;
//            liftIntegral += error;
//            double derivative = error - liftPreviousError;
//            double power = (liftKp * error) + (liftKi * liftIntegral) + (liftKd * derivative);
//            power = Math.max(-1.0, Math.min(1.0, power));
//            liftMotor.setPower(power);
//            liftPreviousError = error;
//
//            telemetry.addData("Lift Target position", liftTargetPosition);
//            telemetry.addData("Lift Current position", currentPosition);
//            telemetry.addData("Lift error", error);
//            telemetry.addData("Lift Power", power);
//            telemetry.addData("Lift Motor current", liftMotor.getCurrent(CurrentUnit.AMPS));

            if (gamepad1.dpad_up) {
                liftTargetPosition = 4000;
            } else if (gamepad1.dpad_left) {
                liftTargetPosition = 400;
            } else if (gamepad1.dpad_down) {
               // if (clawArmTargetPosition < 200) {
                    liftTargetPosition = 0;
                //}
            }
            double positionError = liftTargetPosition - currentPosition;

            double targetVelocity = positionError * 0.0005;
            targetVelocity = Math.max(-MAX_VELOCITY, Math.min(MAX_VELOCITY, targetVelocity));

            double targetAcceleration = (targetVelocity - liftMotor.getVelocity()) / 0.02; // 20 ms loop ?
            targetAcceleration = Math.max(-MAX_ACCELERATION, Math.min(MAX_ACCELERATION, targetAcceleration));

            double feedForwardOutput = elevatorFeedforward.calculate(targetVelocity, targetAcceleration);

            telemetry.addData("Lift Position", currentPosition);
            telemetry.addData("Target Position", liftTargetPosition);
            telemetry.addData("Position Error", positionError);
            telemetry.addData("Lift Velocity", liftMotor.getVelocity());
            telemetry.addData("Target Velocity", targetVelocity);
            telemetry.addData("feedForwardOutput", feedForwardOutput);

            liftMotor.setPower(feedForwardOutput);
        }

//        {
//            double currentPosition = clawArmMotor.getCurrentPosition();
//            double error = clawArmTargetPosition - currentPosition;
//            clawArmIntegral += error;
//            double derivative = error - clawArmPreviousError;
//            double power = (clawArmKp * error) + (clawArmKi * clawArmIntegral) + (clawArmKd * derivative);
//            power = Math.max(-0.40, Math.min(0.25, power));
//            clawArmMotor.setPower(power);
//            clawArmPreviousError = error;
//
//            telemetry.addData("Claw Arm Target position", clawArmTargetPosition);
//            telemetry.addData("Claw Arm Current position", currentPosition);
//            telemetry.addData("Claw Arm error", error);
//            telemetry.addData("Claw Arm Power", power);
//            telemetry.addData("Claw Arm Motor current", clawArmMotor.getCurrent(CurrentUnit.AMPS));
//
//            if (gamepad1.y) {
//                clawArmTargetPosition = 0;// += 10;
//                // clawArmMotor.setPower(0.4);
//            } else if (gamepad1.a) {
//                if (liftTargetPosition > 300) {
//                    clawArmTargetPosition = 495;// -= 10;
//                    //clawArmMotor.setPower(-0.25);
//                }
//            } else {
//                // clawArmMotor.setPower(0);
//            }
//        }

        telemetry.update();
    }


    @Override
    public void stop() {
        liftMotor.setPower(0);
    }
}
