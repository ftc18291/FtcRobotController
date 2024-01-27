package org.firstinspires.ftc.teamcode.mechwarriors.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechwarriors.Utilities;

public class MechRobot extends Robot {

    // Drive motors: 537.7 ticks per revolution
    // Wheels 96mm diameter
    private final static double DRIVE_WHEEL_DIAMETER_MM = 96;
    private final static double DRIVE_WHEEL_DIAMETER_IN = DRIVE_WHEEL_DIAMETER_MM / Utilities.MILLIMETERS_PER_INCH;
    private final static double DRIVE_WHEEL_CIRCUMFERENCE_IN = DRIVE_WHEEL_DIAMETER_IN * Math.PI;
    private final static double DRIVE_WHEEL_TICKS_PER_ROTATION = 537.7;
    private final static double DRIVE_WHEEL_TICKS_PER_ONE_INCH = DRIVE_WHEEL_CIRCUMFERENCE_IN / DRIVE_WHEEL_TICKS_PER_ROTATION;

    // Drive motors
    DcMotor frontRightMotor;
    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;
    DcMotor backRightMotor;

    Claw leftClaw;
    Claw rightClaw;
    LinearSlideLift lift;

    Servo leftSweeperServo;
    Servo rightSweeperServo;

    LEDIndicator sweeperPositionIndicator;

    boolean sweeperStateChanged = false;
    boolean sweepersOpen = false;

    public MechRobot(HardwareMap hardwareMap) {
        super(hardwareMap);
        // Front Left Motor
        frontLeftMotor = hardwareMap.get(DcMotor.class, "front_left_motor");
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        // Front Right Motor
        frontRightMotor = hardwareMap.get(DcMotor.class, "front_right_motor");

        // Back Left Motor
        backLeftMotor = hardwareMap.get(DcMotor.class, "back_left_motor");
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        // Back Right Motor
        backRightMotor = hardwareMap.get(DcMotor.class, "back_right_motor");
        rightSweeperServo = hardwareMap.get(Servo.class, "right_sweeper_servo");
        leftSweeperServo = hardwareMap.get(Servo.class, "left_sweeper_servo");
        leftSweeperServo.setDirection(Servo.Direction.REVERSE);
        rightSweeperServo.scaleRange(0.0, 0.47);  // open, closed
        leftSweeperServo.scaleRange(0.425, 0.95); // closed, open



        sweeperPositionIndicator = new LEDIndicator(hardwareMap);
        sweeperPositionIndicator.setColor(LEDIndicator.LEDColor.GREEN);

        closeSweepers();

        setDriveMotorZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        resetMotorTicks();

        initIMU(hardwareMap);

        leftClaw = new PixelClaw(
                hardwareMap,
                "left_claw_servo",
                "left_claw_extension_servo",
                true,
                0.57, //0.57
                0.65, // 0.65
                0.62, // this is actually down
                1.0); // this is actually up
        rightClaw = new PixelClaw(
                hardwareMap,
                "right_claw_servo",
                "right_claw_extension_servo",
                false,
                0.11, //0.1
                0.335, //0.32
                0,
                0.42); // .5
        lift = new LinearSlideLift(hardwareMap);
    }

    public Claw getLeftClaw() {
        return leftClaw;
    }

    public Claw getRightClaw() {
        return rightClaw;
    }



    public LinearSlideLift getLift() {
        return lift;
    }

    public void drive(double powerFrontRight, double powerFrontLeft, double powerBackLeft, double powerBackRight) {
        frontRightMotor.setPower(powerFrontRight);
        frontLeftMotor.setPower(powerFrontLeft);
        backLeftMotor.setPower(powerBackLeft);
        backRightMotor.setPower(powerBackRight);
    }

    /**
     * Computes the number of drive motor ticks to go the specified distance
     *
     * @param distanceInInches distance in inches
     * @return number of ticks
     */
    @Override
    public int calculateDriveTicks(double distanceInInches) {
        return (int) (distanceInInches / DRIVE_WHEEL_TICKS_PER_ONE_INCH);
    }


    /**
     * Returns the distance the robot has traveled forward or backward in ticks
     *
     * @return the average ticks
     */
    @Override
    public double getDriveTicks() {
        return (frontLeftMotor.getCurrentPosition() +
                backLeftMotor.getCurrentPosition() +
                frontRightMotor.getCurrentPosition() +
                backRightMotor.getCurrentPosition()) / 4.0;
    }

    @Override
    public void drive(double x, double y, double rx) {
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontRightPower = ((y - x - rx) / denominator);
        double frontLeftPower = ((y + x + rx) / denominator);
        double backLeftPower = ((y - x + rx) / denominator);
        double backRightPower = ((y + x - rx) / denominator);
        drive(frontRightPower, frontLeftPower, backLeftPower, backRightPower);
    }


    public double getTranslateDistance() {
        return (Math.abs(frontLeftMotor.getCurrentPosition()) +
                Math.abs(backLeftMotor.getCurrentPosition()) +
                Math.abs(frontRightMotor.getCurrentPosition()) +
                Math.abs(backRightMotor.getCurrentPosition())) / 4.0;
    }

    public String getDriveTicksString() {
        return "fl: " + frontLeftMotor.getCurrentPosition() + "\nbl: " +
                backLeftMotor.getCurrentPosition() + "\nfr: " +
                frontRightMotor.getCurrentPosition() + "\nbr: " +
                backRightMotor.getCurrentPosition();
    }

    public void setDriveMotorZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        frontLeftMotor.setZeroPowerBehavior(zeroPowerBehavior);
        backLeftMotor.setZeroPowerBehavior(zeroPowerBehavior);
        frontRightMotor.setZeroPowerBehavior(zeroPowerBehavior);
        backRightMotor.setZeroPowerBehavior(zeroPowerBehavior);
    }

    @Override
    public void resetMotorTicks() {
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void stop() {
        this.drive(0, 0, 0, 0);
    }

    @Override
    public void openSweepers() {
        sweeperPositionIndicator.setColor(LEDIndicator.LEDColor.RED);
        leftSweeperServo.setPosition(0);
        rightSweeperServo.setPosition(0);
    }

    @Override
    public void closeSweepers() {
        sweeperPositionIndicator.setColor(LEDIndicator.LEDColor.GREEN);
        leftSweeperServo.setPosition(1.0);
        rightSweeperServo.setPosition(1.0);
    }



    public void toggleSweepers(boolean buttonPressed, Telemetry telemetry) {
        if (buttonPressed) {
            sweeperStateChanged = true;
        } else {
            if (sweeperStateChanged) {
                sweepersOpen = !sweepersOpen;
                sweeperStateChanged = false;
            }
        }

        if (sweepersOpen) {
            openSweepers();
        } else {
            closeSweepers();
        }
    }

}
