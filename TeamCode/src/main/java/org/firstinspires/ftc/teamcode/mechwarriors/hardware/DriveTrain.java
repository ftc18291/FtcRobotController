package org.firstinspires.ftc.teamcode.mechwarriors.hardware;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DriveTrain {
    HardwareMap hardwareMap;

    Telemetry telemetry;

    IMU imu;
    DcMotorEx leftFrontMotor;
    DcMotorEx rightFrontMotor;
    DcMotorEx leftRearMotor;
    DcMotorEx rightRearMotor;

    boolean isSlowModeOn = false;
    boolean isSuperSlowModeOn = false;


    public DriveTrain(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        leftFrontMotor = initMotor("leftFrontMotor", true);
        rightFrontMotor = initMotor("rightFrontMotor", false);
        leftRearMotor = initMotor("leftRearMotor", true);
        rightRearMotor = initMotor("rightRearMotor", false);


        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
    }

    public void drive(double x, double y, double rx) {
        x *= 1.1; // Counteract imperfect strafing
        y = -y;

        if (isSlowModeOn && !isSuperSlowModeOn) {
            x *= 0.4;
            y *= 0.4;
            rx *= 0.4;
        }

        if (isSlowModeOn && isSuperSlowModeOn) {
            x *= 0.2;
            y *= 0.2;
            rx *= 0.2;
        }

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontRightPower = ((y - x - rx) / denominator);
        double frontLeftPower = ((y + x + rx) / denominator);
        double backLeftPower = ((y - x + rx) / denominator);
        double backRightPower = ((y + x - rx) / denominator);

        telemetry.addData("Front Left", frontLeftPower);
        telemetry.addData("Rear Left", backLeftPower);
        telemetry.addData("Front Right", frontRightPower);
        telemetry.addData("Rear Right", backRightPower);

        drive(frontRightPower, frontLeftPower, backLeftPower, backRightPower);
    }

    public void drive(double powerFrontRight, double powerFrontLeft, double powerBackLeft, double powerBackRight) {
        rightFrontMotor.setPower(powerFrontRight);
        leftFrontMotor.setPower(powerFrontLeft);
        leftRearMotor.setPower(powerBackLeft);
        rightRearMotor.setPower(powerBackRight);
    }

    public void setSlowMode(boolean isOn) {
        isSlowModeOn = isOn;
    }

    public void setSuperSlowMode(boolean isOn) {
        isSuperSlowModeOn = isOn;
    }

    private DcMotorEx initMotor(String motorName, boolean reverse) {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, motorName);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (reverse) {
            motor.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        return motor;
    }
}
