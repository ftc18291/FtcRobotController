package org.firstinspires.ftc.teamcode.mechwarriors.hardware;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.mechwarriors.Utilities;

public class BackupRobot extends Robot {
    private final static double DRIVE_WHEEL_DIAMETER_MM = 90;
    private final static double DRIVE_WHEEL_DIAMETER_IN = DRIVE_WHEEL_DIAMETER_MM / Utilities.MILLIMETERS_PER_INCH;
    private final static double DRIVE_WHEEL_CIRCUMFERENCE_IN = DRIVE_WHEEL_DIAMETER_IN * Math.PI;
    private final static double DRIVE_WHEEL_TICKS_PER_ROTATION = 28 * 3 * 4;
    private final static double DRIVE_WHEEL_TICKS_PER_ONE_INCH = 26.55;// DRIVE_WHEEL_TICKS_PER_ROTATION / DRIVE_WHEEL_CIRCUMFERENCE_IN;
    DcMotor leftMotor;
    DcMotor rightMotor;

    public BackupRobot(HardwareMap hardwareMap) {
        super(hardwareMap);
        leftMotor = hardwareMap.get(DcMotor.class, "LeftDriveMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "RightDriveMotor");
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        resetMotorTicks();
        resetYaw();
    }

    @Override
    public SparkFunOTOS getSparkFunOTOS() {
        return null;
    }

    @Override
    public void stop() {
        this.drive(0, 0, 0);

    }

    @Override
    public Claw getSampleClaw() {
        return null;
    }


    @Override
    public int calculateDriveTicks(double distanceInInches) {

        return (int) (distanceInInches * DRIVE_WHEEL_TICKS_PER_ONE_INCH);
    }

    public double getDriveTicks() {
        return (leftMotor.getCurrentPosition() +
                rightMotor.getCurrentPosition()) / 2.0;
    }

    @Override
    public void drive(double x, double y, double rx) {
        rightMotor.setPower(y - rx);
        leftMotor.setPower(y + rx);
    }
    @Override
    public void resetMotorTicks() {
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public double getTranslateDistance() {
        return 0;
    }

}
