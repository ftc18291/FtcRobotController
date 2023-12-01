package org.firstinspires.ftc.teamcode.mechwarriors.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.mechwarriors.Utilities;

public class LinearSlideLift {
    private final static double LIFT_MAX_UP_POWER = 0.5;
    private final static double LIFT_MAX_DOWN_POWER = 0.5;
    private final static double LIFT_MIN_TICKS = 0;
    private final static double LIFT_MAX_TICKS = 2700;
    private final static double LIFT_SLOW_ZONE = 500;
    private final static double LIFT_SPOOL_DIAMETER_MM = 34;
    private final static double LIFT_SPOOL_DIAMETER_IN = LIFT_SPOOL_DIAMETER_MM / Utilities.MILLIMETERS_PER_INCH;
    private final static double LIFT_SPOOL_CIRCUMFERENCE_IN = LIFT_SPOOL_DIAMETER_IN * Math.PI;
    private final static double LIFT_MOTOR_TICKS_PER_ROTATION = 28;
    private final static double LIFT_MOTOR_GEAR_RATIO = 20;
    private final static double LIFT_SPOOL_TICKS_PER_ROTATION = LIFT_MOTOR_GEAR_RATIO * LIFT_MOTOR_TICKS_PER_ROTATION;
    private final static double LIFT_SPOOL_TICKS_PER_ONE_INCH = LIFT_SPOOL_CIRCUMFERENCE_IN / LIFT_SPOOL_TICKS_PER_ROTATION;

    // Lift motors
    DcMotor leftLiftMotor;
    DcMotor rightLiftMotor;

    public LinearSlideLift(HardwareMap hardwareMap) {
        // Left Lift Motor
        leftLiftMotor = hardwareMap.get(DcMotor.class, "left_lift_motor");
        leftLiftMotor.setDirection(DcMotor.Direction.REVERSE);
        leftLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Right Lift Motor
        rightLiftMotor = hardwareMap.get(DcMotor.class, "right_lift_motor");
        rightLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void liftArmUp() {
        if (leftLiftMotor.getCurrentPosition() >= LIFT_MAX_TICKS || rightLiftMotor.getCurrentPosition() >= LIFT_MAX_TICKS) {
            liftArmStop();
        } else if (leftLiftMotor.getCurrentPosition() >= (LIFT_MAX_TICKS - LIFT_SLOW_ZONE) || rightLiftMotor.getCurrentPosition() >= (LIFT_MAX_TICKS - LIFT_SLOW_ZONE)) {
            leftLiftMotor.setPower(LIFT_MAX_UP_POWER * 0.5);
            rightLiftMotor.setPower(LIFT_MAX_UP_POWER * 0.5);
        } else {
            leftLiftMotor.setPower(LIFT_MAX_UP_POWER);
            rightLiftMotor.setPower(LIFT_MAX_UP_POWER);
        }
    }

    public void liftArmDown() {
        if (leftLiftMotor.getCurrentPosition() <= LIFT_MIN_TICKS || rightLiftMotor.getCurrentPosition() <= LIFT_MIN_TICKS) {
            liftArmStop();
        } else if (leftLiftMotor.getCurrentPosition() <= LIFT_SLOW_ZONE || rightLiftMotor.getCurrentPosition() <= LIFT_SLOW_ZONE) {
            leftLiftMotor.setPower(-LIFT_MAX_DOWN_POWER * 0.5);
            rightLiftMotor.setPower(-LIFT_MAX_DOWN_POWER * 0.5);
        } else {
            leftLiftMotor.setPower(-LIFT_MAX_DOWN_POWER);
            rightLiftMotor.setPower(-LIFT_MAX_DOWN_POWER);
        }
    }

    public void liftArmStop() {
        leftLiftMotor.setPower(0);
        rightLiftMotor.setPower(0);
    }

    public double getLiftTicks() {
        return (leftLiftMotor.getCurrentPosition() + rightLiftMotor.getCurrentPosition()) / 2.0;
    }

    public double calculateLiftTicks(double heightInInches) {
        return heightInInches / LIFT_SPOOL_TICKS_PER_ONE_INCH;
    }
}

