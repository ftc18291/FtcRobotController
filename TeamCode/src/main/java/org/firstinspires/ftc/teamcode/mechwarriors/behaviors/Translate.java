package org.firstinspires.ftc.teamcode.mechwarriors.behaviors;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechwarriors.hardware.MechRobot;

public class Translate extends Behavior {
    MechRobot robot;
    int heading;
    int distance;
    double speed;

    public Translate(Telemetry telemetry, MechRobot robot, int heading, int distance, double speed) {
        this.robot = robot;
        this.telemetry = telemetry;
        this.name = "Translate = [distance: " + distance + "] [heading: " + heading + "] [speed: " + speed + "]";
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
        if (robot.getTranslateDistance() < distance) {
            double robotHeading = robot.getHeading();
            telemetry.addData("robotHeading", robotHeading);
            double steeringCorrection = (robotHeading - heading) * 0.02;
            telemetry.addData("steeringCorrection", steeringCorrection);
            robot.drive(speed, 0, steeringCorrection);
        } else {
            robot.stop();
            isDone = true;
        }
    }
}
