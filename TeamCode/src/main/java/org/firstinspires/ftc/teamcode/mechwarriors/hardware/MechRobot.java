package org.firstinspires.ftc.teamcode.mechwarriors.hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.mechwarriors.Utilities;

public class MechRobot {


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


    // IMU
    BNO055IMU imu;

    Claw claw;
    LinearSlideLift lift;
    JunctionDetectionSenorArray junctionDetectionSenorArray;

    public MechRobot(HardwareMap hardwareMap) {
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

        setDriveMotorZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        resetMotorTicks();


        initIMU(hardwareMap);

        claw = new EthanClaw(hardwareMap);
        lift = new LinearSlideLift(hardwareMap);
        junctionDetectionSenorArray = new JunctionDetectionSenorArray(hardwareMap);
    }

    void initIMU(HardwareMap hardwareMap) {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.accelerationIntegrationAlgorithm = null;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationData = null;
        parameters.calibrationDataFile = "";
        parameters.loggingEnabled = false;
        parameters.loggingTag = "IMU";
        imu.initialize(parameters);
    }

    public Claw getClaw() {
        return claw;
    }

    public LinearSlideLift getLift() {
        return lift;
    }

    public JunctionDetectionSenorArray getJunctionDetectionSenorArray() {
        return junctionDetectionSenorArray;
    }

    public void drive(double powerFrontRight, double powerFrontLeft, double powerBackLeft, double powerBackRight) {
        frontRightMotor.setPower(powerFrontRight);
        frontLeftMotor.setPower(powerFrontLeft);
        backLeftMotor.setPower(powerBackLeft);
        backRightMotor.setPower(powerBackRight);
    }

    public void mecanumDrive(double x, double y, double rx) {
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontRightPower = ((y - x - rx) / denominator);
        double frontLeftPower = ((y + x + rx) / denominator);
        double backLeftPower = ((y - x + rx) / denominator);
        double backRightPower = ((y + x - rx) / denominator);
        drive(frontRightPower, frontLeftPower, backLeftPower, backRightPower);
    }

    /**
     * Computes the number of drive motor ticks to go the specified distance
     *
     * @param distanceInInches distance in inches
     * @return number of ticks
     */
    public int calculateDriveTicks(double distanceInInches) {
        return (int) (distanceInInches / DRIVE_WHEEL_TICKS_PER_ONE_INCH);
    }

    /**
     * Returns the distance the robot has traveled forward or backward in ticks
     *
     * @return the average ticks
     */
    public double getDriveTicks() {
        return (frontLeftMotor.getCurrentPosition() +
                backLeftMotor.getCurrentPosition() +
                frontRightMotor.getCurrentPosition() +
                backRightMotor.getCurrentPosition()) / 4.0;
    }

    public double getHeading() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
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

    public void stop() {
        this.drive(0, 0, 0, 0);
    }
}
