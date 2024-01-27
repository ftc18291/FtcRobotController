package org.firstinspires.ftc.teamcode.mechwarriors.behaviors;

import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.mechwarriors.hardware.Robot;

public class DriveToDistance extends Behavior {
    Robot robot;
    int heading;
    double distance;
    double speed;
    DistanceSensor distanceSensor;

    public DriveToDistance(Telemetry telemetry, Robot robot, DistanceSensor distanceSensor, int heading, double distance, double speed) {
        this.robot = robot;
        this.telemetry = telemetry;
        this.name = "Drive To Distance = [heading: " + heading + "°] [distance: " + distance + "] [speed: " + speed + "]";
        this.heading = heading;
        this.distance = distance;
        this.speed = speed;
        this.distanceSensor = distanceSensor;
    }

    @Override
    public void start() {
        robot.resetMotorTicks();
        run();
    }

    @Override
    public void run() {
        double currentDistance = distanceSensor.getDistance(DistanceUnit.INCH);
        telemetry.addData("robot currentDistance", currentDistance);
        telemetry.addData("Distance", this.distance);
        if (currentDistance >= this.distance) {
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
