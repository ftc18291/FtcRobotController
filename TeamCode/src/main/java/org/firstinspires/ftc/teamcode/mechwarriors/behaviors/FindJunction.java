package org.firstinspires.ftc.teamcode.mechwarriors.behaviors;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechwarriors.hardware.JunctionDetectionSenorArray;
import org.firstinspires.ftc.teamcode.mechwarriors.hardware.MechRobot;

public class FindJunction extends Behavior {
    MechRobot robot;
    int maxDistance;
    int searchState = 1;

    public FindJunction(Telemetry telemetry, MechRobot robot, int maxDistance) {
        this.robot = robot;
        this.telemetry = telemetry;
        this.name = "Find Junction = [maxDistance: " + maxDistance + "]";
        this.maxDistance = maxDistance;
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
        telemetry.addData("distance", maxDistance);
        // if (ticks <= maxDistance) {
        JunctionDetectionSenorArray.DistanceData distanceData = robot.getJunctionDetectionSenorArray().detect();
        telemetry.addLine(robot.getJunctionDetectionSenorArray().distancesToString());
        double robotHeading = robot.getHeading();
        telemetry.addData("robotHeading", robotHeading);
        double steeringCorrection = 0;
        double speed = 0.07;
        if (distanceData.left > 500 && distanceData.right > 500) {
            // drive straight
        } else {
            if (distanceData.left < 250 && distanceData.right > 250) {
                // turn left
                speed = 0.0;
                steeringCorrection = -0.2;
            } else if (distanceData.left > 250 && distanceData.right < 250) {
                // turn right
                speed = 0.0;
                steeringCorrection = 0.2;
            } else if (distanceData.left < 250 && distanceData.right < 250) {
                // we are done
                isDone = true;
            }
        }
        if (isDone) {
            robot.stop();
            telemetry.addLine("Done finding junction");
        } else {
            telemetry.addData("steeringCorrection", steeringCorrection);
            robot.mecanumDrive(0, speed, steeringCorrection);
        }

        // }
    }
}
