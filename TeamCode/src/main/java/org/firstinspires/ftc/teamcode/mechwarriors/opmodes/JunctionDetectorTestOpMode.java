package org.firstinspires.ftc.teamcode.mechwarriors.opmodes;

import static org.firstinspires.ftc.teamcode.mechwarriors.behaviors.TurnToHeading.almostEqual;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.mechwarriors.AllianceColor;
import org.firstinspires.ftc.teamcode.mechwarriors.SignalSide;
import org.firstinspires.ftc.teamcode.mechwarriors.StartingLocation;
import org.firstinspires.ftc.teamcode.mechwarriors.behaviors.Behavior;
import org.firstinspires.ftc.teamcode.mechwarriors.behaviors.FindJunction;
import org.firstinspires.ftc.teamcode.mechwarriors.hardware.AprilTagSignalDetector;
import org.firstinspires.ftc.teamcode.mechwarriors.hardware.JunctionDetectionSenorArray;
import org.firstinspires.ftc.teamcode.mechwarriors.hardware.MechRobot;
import org.firstinspires.ftc.teamcode.mechwarriors.hardware.SignalDetector;

import java.util.ArrayList;
import java.util.List;

@Autonomous(group = "MechWarriors")
public class JunctionDetectorTestOpMode extends OpMode {

    MechRobot robot;
    SignalDetector signalDetector;

    AllianceColor allianceColor = AllianceColor.BLUE;
    StartingLocation startingLocation = StartingLocation.FRONT;
    SignalSide signalSide = SignalSide.NONE;

    List<Behavior> behaviors = new ArrayList<>();
    int state = 0;

    String runningState = "search1";
    DistanceSensor testSensor;
    Servo testServo;
    double testServoPosition = 0.0;
    double minDistance1 = 999.9;
    double minDistance2 = 999.9;
    double minDistancePosition1 = 0.0;
    double minDistancePosition2 = 0.0;
    double minDistancePosition = 0.0;
    final static double TEST_SERVO_MIN = 0.05;
    final static double TEST_SERVO_MAX = 0.85;
    final static double TEST_SERVO_INCREMENT = 0.005;
    final static double DISTANCE_FROM_ROBOT_CENTER_TO_SENSOR = 23;
    double desiredHeading = 0.0;

    @Override
    public void init() {
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        robot = new MechRobot(hardwareMap);
        signalDetector = new AprilTagSignalDetector(telemetry, hardwareMap);
        telemetry.addLine("Init done");

        testSensor = hardwareMap.get(DistanceSensor.class, "testSensor");
        testServo = hardwareMap.get(Servo.class, "testServo");

        testServo.setPosition(TEST_SERVO_MIN);
        testServoPosition = TEST_SERVO_MIN;
    }

    @Override
    public void init_loop() {
        signalSide = signalDetector.detect();

        robot.getJunctionDetectionSenorArray().detect();
        telemetry.addLine(robot.getJunctionDetectionSenorArray().distancesToString());

        if (gamepad1.y) {
            startingLocation = StartingLocation.BACK;
        } else if (gamepad1.a) {
            startingLocation = StartingLocation.FRONT;
        }

        if (gamepad1.x) {
            allianceColor = AllianceColor.BLUE;
        } else if (gamepad1.b) {
            allianceColor = AllianceColor.RED;
        }

        telemetry.addLine("Select Location and Alliance Color");
        telemetry.addData("Starting Location", startingLocation);
        telemetry.addData("Alliance Color", allianceColor);
        telemetry.addData("Signal Side", signalSide);
        telemetry.addLine(drawStartingLocation());

        telemetry.update();
    }

    @Override
    public void start() {
        // behaviors.add(new DriveHeading(telemetry, robot, 0, robot.calculateDriveTicks(4), 0.25));
        // behaviors.get(0).start();
    }

    @Override
    public void loop() {
        telemetry.addLine("Running program...");
        telemetry.addData("Starting Location", startingLocation);
        telemetry.addData("Alliance Color", allianceColor);
        telemetry.addData("Signal Side", signalSide);

        if (runningState == "search1") {
            testServo.setPosition(testServoPosition);
            try {
                Thread.sleep(50);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            testServoPosition += TEST_SERVO_INCREMENT;
            if (testServoPosition >= TEST_SERVO_MAX) {
                runningState = "search2";
                testServoPosition = TEST_SERVO_MAX;
            }
            telemetry.addData("testServoPosition", testServoPosition);
            double dist = testSensor.getDistance(DistanceUnit.CM);
            telemetry.addData("testSensor", dist);
            if (dist < minDistance1) {
                minDistance1 = dist;
                minDistancePosition1 = testServoPosition;
            }
            telemetry.addData("minDistance1", minDistance1);
        } else if (runningState == "search2") {
            testServo.setPosition(testServoPosition);
            try {
                Thread.sleep(50);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            testServoPosition -= TEST_SERVO_INCREMENT;
            if (testServoPosition <= TEST_SERVO_MIN) {
                runningState = "search3";
            }
            telemetry.addData("testServoPosition", testServoPosition);
            double dist = testSensor.getDistance(DistanceUnit.CM);
            telemetry.addData("testSensor", dist);
            if (dist < minDistance2) {
                minDistance2 = dist;
                minDistancePosition2 = testServoPosition;
            }
            telemetry.addData("minDistance2", minDistance2);
        } else if (runningState == "search3") {
            double minDistance = (minDistance1 + minDistance2) / 2;
            minDistancePosition = (minDistancePosition1 + minDistancePosition2) / 2;
            telemetry.addData("minDistancePosition", minDistancePosition);
            telemetry.addData("minDistance", minDistance);
            testServo.setPosition(minDistancePosition);

            double dist = testSensor.getDistance(DistanceUnit.CM);
            telemetry.addData("current distance", dist);

            double sensorAngleInDegrees = Range.scale(minDistancePosition, TEST_SERVO_MIN, TEST_SERVO_MAX, -70, 60) * -1;

            telemetry.addData("sensorAngle", sensorAngleInDegrees);
            double sensorAngleInRadians = Math.toRadians(sensorAngleInDegrees);
            telemetry.addData("sensorAngleInDegrees", sensorAngleInDegrees);
            double opposite = (Math.sin(sensorAngleInRadians) * minDistance);
            telemetry.addData("opposite", opposite);
            double adjacent = (Math.cos(sensorAngleInRadians) * minDistance);
            telemetry.addData("adjacent", adjacent);
            double newAngleInRadians = Math.atan(opposite / (adjacent + DISTANCE_FROM_ROBOT_CENTER_TO_SENSOR));
            telemetry.addData("newAngleInRadians", newAngleInRadians);
            double newAngleInDegrees = Math.toDegrees(newAngleInRadians);
            telemetry.addData("newAngleInDegrees", newAngleInDegrees);
            desiredHeading = newAngleInDegrees;

            //runningState = "turnToHeading";
        } else if (runningState == "turnToHeading") {
            telemetry.addData("minDistancePosition", minDistancePosition);
            telemetry.addData("desiredHeading", desiredHeading);
            double robotHeading = robot.getHeading();

            if (!almostEqual(robotHeading, desiredHeading, 0.75)) {
                telemetry.addData("robotHeading", robotHeading);
                double steeringCorrection = (robotHeading - desiredHeading) * 0.03;
                telemetry.addData("steeringCorrection", steeringCorrection);
                robot.mecanumDrive(0, 0, steeringCorrection);
            } else {
                robot.stop();
                runningState = "done";
            }
        }

        if (state < behaviors.size()) {
            if (!behaviors.get(state).isDone()) {
                telemetry.addData("Running behavior", behaviors.get(state).getName());
                behaviors.get(state).run();
            } else {
                state++;
                if (state < behaviors.size()) {
                    behaviors.get(state).start();
                }
            }
        } else {
            telemetry.addLine("Program done");
            this.stop();
        }
    }

    private String drawStartingLocation() {
        StringBuilder sb = new StringBuilder();
        sb.append("\n  JUDGES\n");
        sb.append("|--------|\n");
        if (startingLocation == StartingLocation.BACK) {
            if (allianceColor == AllianceColor.BLUE) {
                sb.append("| X      |\n");
            } else {
                sb.append("|      X |\n");
            }
        } else {
            sb.append("|        |\n");
        }
        sb.append("|        |\n");
        if (startingLocation == StartingLocation.FRONT) {
            if (allianceColor == AllianceColor.BLUE) {
                sb.append("| X      |\n");
            } else {
                sb.append("|      X |\n");
            }
        } else {
            sb.append("|        |\n");
        }
        sb.append("|--------|\n");
        sb.append(" AUDIENCE\n");
        return sb.toString();
    }
}
