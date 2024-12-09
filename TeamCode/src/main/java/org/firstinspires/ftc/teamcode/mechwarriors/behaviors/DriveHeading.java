package org.firstinspires.ftc.teamcode.mechwarriors.behaviors;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechwarriors.hardware.Robot;

public class DriveHeading extends Behavior {
    Robot robot;
    int heading;
    int distance;
    double speed;

    public DriveHeading(Telemetry telemetry, Robot robot, int heading, double distance, double speed) {
        this.robot = robot;
        this.telemetry = telemetry;
        this.name = "Drive Heading = [heading: " + heading + "°] [distance: " + distance + "] [speed: " + speed + "]";
        this.heading = heading;
        this.distance = robot.calculateDriveTicks(distance);
        this.speed = speed;
    }

    @Override
    public void start() {
        robot.resetMotorTicks();
        robot.getSparkFunOTOS().resetTracking();
        run();
    }

    @Override
    public void run() {
        double ticks = robot.getDriveTicks();
        telemetry.addData("robot ticks", ticks);
        telemetry.addData("distance", distance);
        SparkFunOTOS.Pose2D otosPos = robot.getSparkFunOTOS().getPosition();
        telemetry.addData("otosPos", "x: " + otosPos.x + ", y: " + otosPos.y);
        if (ticks <= distance) {
            double robotHeading = robot.getHeading();
            telemetry.addData("robotHeading", robotHeading);
            double steeringCorrection = (robotHeading - heading) * 0.02;
            telemetry.addData("steeringCorrection", steeringCorrection);
            robot.drive(0, speed, steeringCorrection);
        } else {
            robot.stop();
            isDone = true;
        }
    }
}
