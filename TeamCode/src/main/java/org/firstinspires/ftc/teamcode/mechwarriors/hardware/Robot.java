package org.firstinspires.ftc.teamcode.mechwarriors.hardware;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;

public abstract class Robot {
    IMU imu;
    SparkFunOTOS sparkFunOTOS;

    private LinearSlideLift lift;
    public Robot(HardwareMap hardwareMap) {
        initIMU(hardwareMap);
    }

    void initIMU(HardwareMap hardwareMap) {
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(parameters);
    }

    public abstract SparkFunOTOS getSparkFunOTOS();

    public LinearSlideLift getLift() {return lift;}

    public abstract int calculateDriveTicks(double distanceInInches);

    public abstract double getDriveTicks();

    public abstract void resetMotorTicks();

    public abstract double getTranslateDistance();

    public double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public double getPitchAngle() {
        return imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES);
    }

    public double getRollAngle() {
        return imu.getRobotYawPitchRollAngles().getRoll(AngleUnit.DEGREES);
    }

    public void resetYaw() {
        imu.resetYaw();
    }

    public abstract void drive(double x, double y, double rx);

    public abstract void stop();

    public abstract Claw getSampleClaw();
}
