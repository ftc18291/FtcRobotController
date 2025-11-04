package org.firstinspires.ftc.teamcode.mechwarriors.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp
@Config
public class ArtifactSorterTester extends OpMode {

    DcMotorEx sorterMotor;

    String sorterMode = "";
    Boolean sorterResetButtonPressed = false;

    DigitalChannel sorterTouchSensor;

    private static final double TICKS_PER_ROTATION = 587.04;

    private double currentTicksTarget = 0;

    public static double MOTOR_SPEED = 0.3;
    public static  double NEW_P = 20;
    public static  double NEW_I = 10;
    public static  double NEW_D = 0.1;


    @Override
    public void init() {
        sorterMotor = hardwareMap.get(DcMotorEx.class, "sorterMotor");
        sorterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sorterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sorterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sorterTouchSensor = hardwareMap.get(DigitalChannel.class, "sorterTouchSensor");

        PIDFCoefficients pidOrig = sorterMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);

        PIDFCoefficients pidNew = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, 0);
        sorterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidNew);

        PIDFCoefficients pidModified = sorterMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("P,I,D (orig)", "%.04f, %.04f, %.0f, %.04f",
                pidOrig.p, pidOrig.i, pidOrig.d, pidOrig.f);
        telemetry.addData("P,I,D (modified)", "%.04f, %.04f, %.04f, %.04f",
                pidModified.p, pidModified.i, pidModified.d, pidModified.f);
    }

    @Override
    public void loop() {
        PIDFCoefficients pidNew = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, 0);
        sorterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidNew);

        telemetry.addData("sorterTouchSensor", !sorterTouchSensor.getState());
        telemetry.addData("sorterMotor ticks", sorterMotor.getCurrentPosition());
        telemetry.addData("currentTicksTarget", currentTicksTarget);

        if (gamepad1.x) {
            sorterResetButtonPressed = true;
        }
        if (!gamepad1.x && sorterResetButtonPressed) {
            sorterResetButtonPressed = false;
            currentTicksTarget = currentTicksTarget + (TICKS_PER_ROTATION / 3);
            sorterMotor.setTargetPosition((int) currentTicksTarget);
            sorterMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sorterMotor.setPower(MOTOR_SPEED);
        }


        if (gamepad1.a) {
            currentTicksTarget = currentTicksTarget + (TICKS_PER_ROTATION * 50);
            sorterMotor.setTargetPosition((int) currentTicksTarget);
            sorterMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sorterMotor.setPower(MOTOR_SPEED);
        }

        if (gamepad1.b) {
            //currentTicksTarget = currentTicksTarget + (TICKS_PER_ROTATION * 1);
            sorterMotor.setTargetPosition((int) TICKS_PER_ROTATION * 1);
            sorterMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sorterMotor.setPower(MOTOR_SPEED);
        }
//
//        if (gamepad1.x) {
//            currentTicksTarget = currentTicksTarget + (TICKS_PER_ROTATION / 3);
//            sorterMotor.setTargetPosition((int) currentTicksTarget);
//            sorterMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            sorterMotor.setPower(0.2);
//        }
    }
}
