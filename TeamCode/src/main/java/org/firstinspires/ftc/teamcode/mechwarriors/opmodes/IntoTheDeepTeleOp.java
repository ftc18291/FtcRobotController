package org.firstinspires.ftc.teamcode.mechwarriors.opmodes;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.mechwarriors.Utilities;

@TeleOp(group = "IntoTheDeep", name = "TeleOp")
public class IntoTheDeepTeleOp extends OpMode {

    DcMotorEx clawArmMotor;
    DcMotorEx liftMotor;

    Servo rightHangServo;
    Servo leftHangServo;
    DcMotorEx rightHangMotor;
    DcMotorEx leftHangMotor;

    Servo clawServo;

    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backRightMotor;

    SparkFunOTOS sparkFunOTOS;

    int liftTicks = 0;
    boolean slowMode = false;

    @Override
    public void init() {
        sparkFunOTOS = hardwareMap.get(SparkFunOTOS.class, "sparkFunOTOS");

        // All the configuration for the OTOS is done in this helper method, check it out!
        configureOtos();

        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        clawServo = hardwareMap.get(Servo.class, "clawServo");
        clawArmMotor = hardwareMap.get(DcMotorEx.class, "armLiftMotor");
        clawArmMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        clawArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightHangServo = hardwareMap.get(Servo.class, "rightHangServo");
        leftHangServo = hardwareMap.get(Servo.class, "leftHangServo");
        leftHangServo.setDirection(Servo.Direction.REVERSE);

        rightHangMotor = hardwareMap.get(DcMotorEx.class, "rightHangMotor");
        leftHangMotor = hardwareMap.get(DcMotorEx.class, "leftHangMotor");
        rightHangMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftHangMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slowMode = true;
    }

    @Override
    public void loop() {
        SparkFunOTOS.Pose2D pos = sparkFunOTOS.getPosition();

        // Log the position to the telemetry
        telemetry.addData("X coordinate", pos.x);
        telemetry.addData("Y coordinate", pos.y);
        telemetry.addData("Heading angle", pos.h);

        ////////////////////////////////////////////

        if (gamepad1.left_bumper) {
            slowMode = true;
        }
        if (gamepad1.right_bumper) {
            slowMode = false;
        }

        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;

        y = Utilities.squareInputWithSign(y);
        x = Utilities.squareInputWithSign(x);
        rx = Utilities.squareInputWithSign(rx);

        if (slowMode) {
            x = x * 0.4;
            y = y * 0.4;
            rx = rx * 0.4;
        }

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);

        telemetry.addData("Front Left Motor Ticks", frontLeftMotor.getCurrentPosition());

        /////////////////////////////////////////

        if (gamepad1.back) {
            if (gamepad1.y) {
                sparkFunOTOS.resetTracking();
                frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                // 3.19 inches
                while (frontLeftMotor.getCurrentPosition()  < 60) {
                    // do nothing
                    frontLeftMotor.setPower(0.2);
                    backLeftMotor.setPower(0.2);
                    frontRightMotor.setPower(0.2);
                    backRightMotor.setPower(0.2);
                    this.resetRuntime();
                }
                frontLeftMotor.setPower(0);
                backLeftMotor.setPower(0);
                frontRightMotor.setPower(0);
                backRightMotor.setPower(0);
            }
            if (gamepad1.x) {
                // back
                leftHangServo.setPosition(0.85);
                rightHangServo.setPosition(1.0);
            } else if (gamepad1.b) {
                // front
                leftHangServo.setPosition(0.55);
                rightHangServo.setPosition(0.7);
            }
        }

        if (gamepad1.dpad_up) {
            telemetry.addData("hang motors up", "");
            leftHangMotor.setPower(1);
            rightHangMotor.setPower(-1);
        } else if (gamepad1.dpad_down) {
            telemetry.addData("hang motors down", "");
            leftHangMotor.setPower(-1);
            rightHangMotor.setPower(1);
        } else {
            leftHangMotor.setPower(0);
            rightHangMotor.setPower(0);
        }

//
//        if (gamepad1.y) {
//            telemetry.addData("right hang motor up", "");
//            rightHangMotor.setPower(1);
//        } else if (gamepad1.a) {
//            telemetry.addData("right hang motor down", "");
//            rightHangMotor.setPower(-1);
//        } else {
//            rightHangMotor.setPower(0);
//        }

        ////////////////////////////////////////////

        if (gamepad2.dpad_down) {
            clawArmMotor.setPower(0.4);
        } else if (gamepad2.dpad_up) {
            clawArmMotor.setPower(-0.25);
        } else {
            clawArmMotor.setPower(0);
        }
        telemetry.addData("ClawArmMotor Position", clawArmMotor.getCurrentPosition());

        /////////////////////////////////////////

        liftTicks = liftMotor.getCurrentPosition();
        telemetry.addData("lift ticks", liftTicks);
        if (gamepad2.a) {
            liftMotor.setPower(-1);
            if (liftTicks <= 0) {
                liftMotor.setPower(0);
            }
        } else if (gamepad2.b /*&& liftTicks >= 0*/) {
            liftMotor.setPower(1);
            if (liftTicks >= 7500) {
                liftMotor.setPower(0);
            }
        } else {
            liftMotor.setPower(0);
        }
        telemetry.addData("LiftMotor " +
                "Position", clawArmMotor.getCurrentPosition());

        /////////////////////////////////////////

        // claw close
        if (gamepad2.x) {
            clawServo.setPosition(0.56);
        }
        // claw open
        if (gamepad2.y) {
            clawServo.setPosition(0.25);
        }
    }

    private void configureOtos() {
        telemetry.addLine("Configuring OTOS...");
        telemetry.update();

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

        telemetry.addLine("OTOS configured! Press start to get position data!");
        telemetry.addLine();
        telemetry.addLine(String.format("OTOS Hardware Version: v%d.%d", hwVersion.major, hwVersion.minor));
        telemetry.addLine(String.format("OTOS Firmware Version: v%d.%d", fwVersion.major, fwVersion.minor));
        telemetry.update();
    }
}
