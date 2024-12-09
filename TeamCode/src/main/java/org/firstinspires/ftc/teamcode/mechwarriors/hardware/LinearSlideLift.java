package org.firstinspires.ftc.teamcode.mechwarriors.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.mechwarriors.Utilities;

public class LinearSlideLift {
    private final static double LIFT_MAX_UP_POWER = 1.0;
    private final static double LIFT_MAX_DOWN_POWER = 1.0;
    private final static double LIFT_MIN_TICKS = 0;
    private final static double LIFT_MAX_TICKS = 7500;
    private final static double LIFT_SLOW_ZONE = 500;
    private final static double LIFT_SPOOL_DIAMETER_MM = 25;
    private final static double LIFT_SPOOL_DIAMETER_IN = LIFT_SPOOL_DIAMETER_MM / Utilities.MILLIMETERS_PER_INCH;
    private final static double LIFT_SPOOL_CIRCUMFERENCE_IN = LIFT_SPOOL_DIAMETER_IN * Math.PI;
    private final static double LIFT_MOTOR_TICKS_PER_ROTATION = 28;
    private final static double LIFT_MOTOR_GEAR_RATIO = 100;
    private final static double LIFT_SPOOL_TICKS_PER_ROTATION = LIFT_MOTOR_GEAR_RATIO * LIFT_MOTOR_TICKS_PER_ROTATION;
    private final static double LIFT_SPOOL_TICKS_PER_ONE_INCH = LIFT_SPOOL_CIRCUMFERENCE_IN / LIFT_SPOOL_TICKS_PER_ROTATION;

    // Lift motor
    DcMotor liftMotor;

    public LinearSlideLift(HardwareMap hardwareMap) {
        // Lift Motor
        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        liftMotor.setDirection(DcMotor.Direction.REVERSE);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void liftArmUp() {
        if (liftMotor.getCurrentPosition() >= LIFT_MAX_TICKS) {
            liftArmStop();
        } else if (liftMotor.getCurrentPosition() >= (LIFT_MAX_TICKS - LIFT_SLOW_ZONE)) {
            liftMotor.setPower(LIFT_MAX_UP_POWER * 0.5);
        } else {
            liftMotor.setPower(LIFT_MAX_UP_POWER);
        }
    }

    public void liftArmDown() {
        if (liftMotor.getCurrentPosition() <= LIFT_MIN_TICKS) {
            liftArmStop();
        } else if (liftMotor.getCurrentPosition() <= LIFT_SLOW_ZONE) {
            liftMotor.setPower(-LIFT_MAX_DOWN_POWER * 0.5);
        } else {
            liftMotor.setPower(-LIFT_MAX_DOWN_POWER);
        }
    }

    public void liftArmStop() {
        liftMotor.setPower(0);
    }

    public double getLiftTicks() {
        return liftMotor.getCurrentPosition();
    }

    public double calculateLiftTicks(double heightInInches) {
        return heightInInches / LIFT_SPOOL_TICKS_PER_ONE_INCH;
    }
}

