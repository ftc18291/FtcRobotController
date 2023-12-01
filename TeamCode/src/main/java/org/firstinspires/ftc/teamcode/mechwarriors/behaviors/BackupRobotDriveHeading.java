package org.firstinspires.ftc.teamcode.mechwarriors.behaviors;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechwarriors.hardware.BackupRobot;
import org.firstinspires.ftc.teamcode.mechwarriors.hardware.Robot;

public class BackupRobotDriveHeading extends Behavior {
    double speed;
    double distance;
    int heading;
    Telemetry telemetry;
    Robot backupRobot;

    public BackupRobotDriveHeading(Robot robot, Telemetry telemetry, int heading, int distance, double speed) {
        this.speed = speed;
        this.distance = robot.calculateDriveTicks(distance);
        this.heading = heading;
        this.telemetry = telemetry;
        backupRobot = robot;

    }

    @Override
    public void start() {
        backupRobot.resetMotorTicks();
        this.run();

    }

    @Override
    public void run() {
        telemetry.addData("ticks", distance);
        double ticks = backupRobot.getDriveTicks();
        // telemetry.addData("robot ticks", ticks);
        //telemetry.addData("distance", distance);
        if (ticks <= distance) {
            double robotHeading = backupRobot.getHeading();
            telemetry.addData("robotHeading", robotHeading);
            double steeringCorrection = (robotHeading - heading) * 0.01;
            telemetry.addData("steeringCorrection", steeringCorrection);
            backupRobot.drive(0, speed, steeringCorrection);
        } else {
            backupRobot.stop();
            isDone = true;
        }

    }
}
