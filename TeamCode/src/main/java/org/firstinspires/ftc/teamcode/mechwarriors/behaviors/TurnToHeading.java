package org.firstinspires.ftc.teamcode.mechwarriors.behaviors;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechwarriors.hardware.Robot;

public class TurnToHeading extends Behavior {
    Robot robot;
    int desiredHeading;
    double turningSpeed;

    public TurnToHeading(Telemetry telemetry, Robot robot, int heading, double turningSpeed) {
        this.robot = robot;
        this.telemetry = telemetry;
        this.name = "Turn to Heading = [heading: " + heading + "°]";
        this.desiredHeading = heading;
        this.turningSpeed = turningSpeed;
    }

    @Override
    public void start() {
        run();
    }

    @Override
    public void run() {
        double robotHeading = robot.getHeading();

        if (!almostEqual(robotHeading, desiredHeading, 1.0)) {
            telemetry.addData("robotHeading", robotHeading);
            double steeringCorrection = (robotHeading - desiredHeading) * turningSpeed;
            telemetry.addData("steeringCorrection", steeringCorrection);
            robot.drive(0, 0, steeringCorrection);
        } else {
            robot.stop();
            isDone = true;
        }
    }

    public static boolean almostEqual(double a, double b, double difference) {
        return Math.abs(a - b) < difference;
    }
}
