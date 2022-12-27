package org.firstinspires.ftc.teamcode.mechwarriors.behaviors;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechwarriors.hardware.MechRobot;

public class DriveHeading extends Behavior {
    MechRobot robot;
    int heading;
    int distance;
    double speed;

    public DriveHeading(Telemetry telemetry, MechRobot robot, int heading, int distance, double speed) {
        this.robot = robot;
        this.telemetry = telemetry;
        this.name = "Drive Heading = [heading: " + heading + "°] [distance: " + distance + "] [speed: " + speed + "]";
        this.heading = -heading;
        this.distance = distance;
        this.speed = speed;
    }

    @Override
    public void start() {
        robot.resetMotorTicks();
        run();
    }

    @Override
    public void run() {
        double ticks = robot.getDriveTicks();
        telemetry.addData("robot ticks", ticks);
        telemetry.addData("distance", distance);
        if (ticks <= distance) {
            double robotHeading = robot.getHeading();
            telemetry.addData("robotHeading", robotHeading);
            double steeringCorrection = (robotHeading - heading) * 0.02;
            telemetry.addData("steeringCorrection", steeringCorrection);
            robot.mecanumDrive(0, speed, steeringCorrection);
        } else {
            robot.stop();
            isDone = true;
        }
    }
}
