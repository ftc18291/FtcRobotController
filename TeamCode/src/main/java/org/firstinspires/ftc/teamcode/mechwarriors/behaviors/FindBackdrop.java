package org.firstinspires.ftc.teamcode.mechwarriors.behaviors;

import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.mechwarriors.hardware.MechRobot;
import org.firstinspires.ftc.teamcode.mechwarriors.hardware.Robot;

public class FindBackdrop extends Behavior {
    Robot robot;
    int sensorDistance;
    int maxDistance;
    DistanceSensor leftDistanceSensor;
    DistanceSensor rightDistanceSensor;
    int searchState = 1;
    double heading;

    public FindBackdrop(Telemetry telemetry, Robot robot, DistanceSensor leftDistanceSensor, DistanceSensor rightDistanceSensor, int sensorDistance, int maxDistanceInTicks, double heading) {
        this.robot = robot;
        this.telemetry = telemetry;
        this.name = "Find Backdrop = [distance: " + maxDistance + "]";
        this.sensorDistance = sensorDistance;
        this.maxDistance = robot.calculateDriveTicks(maxDistanceInTicks);
        this.leftDistanceSensor = leftDistanceSensor;
        this.rightDistanceSensor = rightDistanceSensor;
        this.heading = heading;
    }

    @Override
    public void start() {
        robot.resetMotorTicks();
        run();
    }

    @Override
    public void run() {
        double ticks = robot.getDriveTicks();
        double leftDistance = leftDistanceSensor.getDistance(DistanceUnit.INCH);
        double rightDistance = rightDistanceSensor.getDistance(DistanceUnit.INCH);
        telemetry.addData("robot ticks", ticks);
        telemetry.addData("distance", maxDistance);
        telemetry.addData("left distance", leftDistanceSensor.getDistance(DistanceUnit.INCH));
        telemetry.addData("right distance", rightDistanceSensor.getDistance(DistanceUnit.INCH));

        double speed = 0.10;
        if (leftDistance > sensorDistance && rightDistance > sensorDistance && ticks <= maxDistance) {
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

