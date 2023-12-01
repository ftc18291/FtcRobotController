package org.firstinspires.ftc.teamcode.mechwarriors.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.mechwarriors.hardware.LEDIndicator;

@TeleOp
public class CenterStageBackupRobotOpMode extends OpMode {
    DcMotor leftDriveMotor;
    DcMotor rightDriveMotor;
    DistanceSensor opticalDistanceSensor;
    TouchSensor touchSensor;
    TouchSensor magneticSensor;
    ColorSensor colorSensor;
    LEDIndicator lightIndicator;


    @Override
    public void init() {
        rightDriveMotor = hardwareMap.get(DcMotor.class, "RightDriveMotor");
        leftDriveMotor = hardwareMap.get(DcMotor.class, "LeftDriveMotor");
        opticalDistanceSensor = hardwareMap.get(DistanceSensor.class, "OpticalDistanceSensor");
        touchSensor = hardwareMap.get(TouchSensor.class, "TouchSensor");
        magneticSensor = hardwareMap.get(TouchSensor.class, "MagneticSensor");
        colorSensor = hardwareMap.get(ColorSensor.class, "ColorSensor");
        lightIndicator = new LEDIndicator(hardwareMap);
        leftDriveMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        double speedy = -gamepad1.left_stick_y;
        double speedx = -gamepad1.left_stick_x;
        telemetry.addData("speedy", speedy);
        telemetry.addData("distance", opticalDistanceSensor.getDistance(DistanceUnit.CM));
        telemetry.addData("touch", touchSensor.isPressed());
        telemetry.addData("magnetic", magneticSensor.isPressed());
        telemetry.addData("red", colorSensor.red());
        telemetry.addData("blue", colorSensor.blue());
        telemetry.addData("green", colorSensor.green());
        rightDriveMotor.setPower(speedy + speedx);
        leftDriveMotor.setPower(speedy - speedx);
        if (gamepad1.dpad_up) {
            lightIndicator.setColor(LEDIndicator.LEDColor.RED);
        } else if (gamepad1.dpad_down) {
            lightIndicator.setColor(LEDIndicator.LEDColor.GREEN);
        } else if (gamepad1.dpad_right) {
            lightIndicator.setColor(LEDIndicator.LEDColor.AMBER);
        } else if (gamepad1.dpad_left) {
            lightIndicator.setColor(LEDIndicator.LEDColor.OFF);
        }

        telemetry.addData("leftMotor", leftDriveMotor.getCurrentPosition());
        telemetry.addData("rightMotor", rightDriveMotor.getCurrentPosition());

    }
}
