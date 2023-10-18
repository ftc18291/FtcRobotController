package org.firstinspires.ftc.teamcode.mechwarriors.opmodes;

import android.hardware.Sensor;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class CenterStageBackupRobotOpMode extends OpMode {
    DcMotor leftDriveMotor;
    DcMotor rightDriveMotor;
    DistanceSensor opticalDistanceSensor;


    @Override
    public void init() {
        rightDriveMotor = hardwareMap.get(DcMotor.class, "RightDriveMotor");
        leftDriveMotor = hardwareMap.get(DcMotor.class, "LeftDriveMotor");
        opticalDistanceSensor = hardwareMap.get(DistanceSensor.class, "OpticalDistanceSensor");
        leftDriveMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        double speedy = -gamepad1.left_stick_y;
        double speedx = -gamepad1.left_stick_x;
        telemetry.addData("speedy", speedy);
        telemetry.addData("distance", opticalDistanceSensor.getDistance(DistanceUnit.CM));
        rightDriveMotor.setPower(speedy + speedx);
        leftDriveMotor.setPower(speedy - speedx);


    }
}
