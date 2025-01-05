package org.firstinspires.ftc.teamcode.mechwarriors.hardware;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.mechwarriors.Utilities;

public class IntoTheDeepRobot extends Robot {

    // Drive motors: 537.7 ticks per revolution
    // Wheels 96mm diameter
    private final static double DRIVE_WHEEL_DIAMETER_MM = 96;
    private final static double DRIVE_WHEEL_DIAMETER_IN = DRIVE_WHEEL_DIAMETER_MM / Utilities.MILLIMETERS_PER_INCH;
    private final static double DRIVE_WHEEL_CIRCUMFERENCE_IN = DRIVE_WHEEL_DIAMETER_IN * Math.PI;
    private final static double DRIVE_WHEEL_TICKS_PER_ROTATION = 537.7;
    private final static double DRIVE_WHEEL_TICKS_PER_ONE_INCH = DRIVE_WHEEL_CIRCUMFERENCE_IN / DRIVE_WHEEL_TICKS_PER_ROTATION;

    SparkFunOTOS sparkFunOTOS;

    // Drive motors
    DcMotor frontRightMotor;
    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;
    DcMotor backRightMotor;

    Claw sampleClaw;
    LinearSlideLift lift;

    public IntoTheDeepRobot(HardwareMap hardwareMap) {
        super(hardwareMap);

//        sparkFunOTOS = hardwareMap.get(SparkFunOTOS.class, "sparkFunOTOS");




        initIMU(hardwareMap);
        //configureOtos();

        sampleClaw = new SampleClaw(
                hardwareMap,
                "clawServo",
                true,
                0.25,
                0.54);
    }

    public Claw getSampleClaw() {
        return sampleClaw;
    }

    public LinearSlideLift getLift() {
        return lift;
    }

    @Override
    public SparkFunOTOS getSparkFunOTOS() { return sparkFunOTOS; }

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

    private void configureOtos() {
//        telemetry.addLine("Configuring OTOS...");
//        telemetry.update();

        // Set the desired units for linear and angular measurements. Can be either
        // meters or inches for linear, and radians or degrees for angular. If not
        // set, the default is inches and degrees. Note that this setting is not
        // persisted in the sensor, so you need to set at the start of all your
        // OpModes if using the non-default value.
        // myOtos.setLinearUnit(DistanceUnit.METER);
        sparkFunOTOS.setLinearUnit(DistanceUnit.INCH);
        // myOtos.setAngularUnit(AnguleUnit.RADIANS);
        sparkFunOTOS.setAngularUnit(AngleUnit.DEGREES);

        // Assuming you've mounted your sensor to a robot and it's not centered,
        // you can specify the offset for the sensor relative to the center of the
        // robot. The units default to inches and degrees, but if you want to use
        // different units, specify them before setting the offset! Note that as of
        // firmware version 1.0, these values will be lost after a power cycle, so
        // you will need to set them each time you power up the sensor. For example, if
        // the sensor is mounted 5 inches to the left (negative X) and 10 inches
        // forward (positive Y) of the center of the robot, and mounted 90 degrees
        // clockwise (negative rotation) from the robot's orientation, the offset
        // would be {-5, 10, -90}. These can be any value, even the angle can be
        // tweaked slightly to compensate for imperfect mounting (eg. 1.3 degrees).
        // 48mm left = 1.89 inches
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(1.89, 0, 0);
        sparkFunOTOS.setOffset(offset);

        // Here we can set the linear and angular scalars, which can compensate for
        // scaling issues with the sensor measurements. Note that as of firmware
        // version 1.0, these values will be lost after a power cycle, so you will
        // need to set them each time you power up the sensor. They can be any value
        // from 0.872 to 1.127 in increments of 0.001 (0.1%). It is recommended to
        // first set both scalars to 1.0, then calibrate the angular scalar, then
        // the linear scalar. To calibrate the angular scalar, spin the robot by
        // multiple rotations (eg. 10) to get a precise error, then set the scalar
        // to the inverse of the error. Remember that the angle wraps from -180 to
        // 180 degrees, so for example, if after 10 rotations counterclockwise
        // (positive rotation), the sensor reports -15 degrees, the required scalar
        // would be 3600/3585 = 1.004. To calibrate the linear scalar, move the
        // robot a known distance and measure the error; do this multiple times at
        // multiple speeds to get an average, then set the linear scalar to the
        // inverse of the error. For example, if you move the robot 100 inches and
        // the sensor reports 103 inches, set the linear scalar to 100/103 = 0.971
        sparkFunOTOS.setLinearScalar(1.068); //93.6   100/93.6
        sparkFunOTOS.setAngularScalar(0.998); // -8.421, 7.000   3600/3607.5

        // The IMU on the OTOS includes a gyroscope and accelerometer, which could
        // have an offset. Note that as of firmware version 1.0, the calibration
        // will be lost after a power cycle; the OTOS performs a quick calibration
        // when it powers up, but it is recommended to perform a more thorough
        // calibration at the start of all your OpModes. Note that the sensor must
        // be completely stationary and flat during calibration! When calling
        // calibrateImu(), you can specify the number of samples to take and whether
        // to wait until the calibration is complete. If no parameters are provided,
        // it will take 255 samples and wait until done; each sample takes about
        // 2.4ms, so about 612ms total
        sparkFunOTOS.calibrateImu();

        // Reset the tracking algorithm - this resets the position to the origin,
        // but can also be used to recover from some rare tracking errors
        sparkFunOTOS.resetTracking();

        // After resetting the tracking, the OTOS will report that the robot is at
        // the origin. If your robot does not start at the origin, or you have
        // another source of location information (eg. vision odometry), you can set
        // the OTOS location to match and it will continue to track from there.
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        sparkFunOTOS.setPosition(currentPosition);

        // Get the hardware and firmware version
        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        sparkFunOTOS.getVersionInfo(hwVersion, fwVersion);

//        telemetry.addLine("OTOS configured! Press start to get position data!");
//        telemetry.addLine();
//        telemetry.addLine(String.format("OTOS Hardware Version: v%d.%d", hwVersion.major, hwVersion.minor));
//        telemetry.addLine(String.format("OTOS Firmware Version: v%d.%d", fwVersion.major, fwVersion.minor));
//        telemetry.update();
    }
}
