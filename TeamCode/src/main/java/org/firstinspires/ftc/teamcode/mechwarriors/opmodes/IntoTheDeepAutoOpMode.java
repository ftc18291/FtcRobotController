package org.firstinspires.ftc.teamcode.mechwarriors.opmodes;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechwarriors.AllianceColor;
import org.firstinspires.ftc.teamcode.mechwarriors.LiftHeight;
import org.firstinspires.ftc.teamcode.mechwarriors.StartingLocation;
import org.firstinspires.ftc.teamcode.mechwarriors.behaviors.Behavior;
import org.firstinspires.ftc.teamcode.mechwarriors.behaviors.CloseClaw;
import org.firstinspires.ftc.teamcode.mechwarriors.behaviors.DriveHeading;
import org.firstinspires.ftc.teamcode.mechwarriors.behaviors.LowerLift;
import org.firstinspires.ftc.teamcode.mechwarriors.behaviors.OpenClaw;
import org.firstinspires.ftc.teamcode.mechwarriors.behaviors.RaiseLift;
import org.firstinspires.ftc.teamcode.mechwarriors.behaviors.ReverseHeading;
import org.firstinspires.ftc.teamcode.mechwarriors.behaviors.Translate;
import org.firstinspires.ftc.teamcode.mechwarriors.behaviors.TurnToHeading;
import org.firstinspires.ftc.teamcode.mechwarriors.behaviors.Wait;
import org.firstinspires.ftc.teamcode.mechwarriors.hardware.Claw;
import org.firstinspires.ftc.teamcode.mechwarriors.hardware.IntoTheDeepRobot;
import org.firstinspires.ftc.teamcode.mechwarriors.hardware.Robot;

import java.util.ArrayList;
import java.util.List;

@Autonomous(group = "IntoTheDeep", name = "Auto OpMode")
public class IntoTheDeepAutoOpMode extends OpMode {

    Robot robot;

    List<Behavior> behaviors = new ArrayList<>();
    int state = 0;
    AllianceColor allianceColor = AllianceColor.BLUE;
    StartingLocation startingLocation = StartingLocation.LEFT;
    int waitTime = 0;
    boolean dpaddownPressed = false;
    boolean dpadupPressed = false;
    Claw liftClaw;

    @Override
    public void init() {
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        robot = new IntoTheDeepRobot(hardwareMap);
        liftClaw = robot.getSampleClaw();
        liftClaw.close();
    }

    @Override
    public void init_loop() {
        if (gamepad1.dpad_down) {
            dpaddownPressed = true;
        } else {
            if (dpaddownPressed) {
                waitTime--;
                dpaddownPressed = false;
            }
        }
        if (gamepad1.dpad_up) {
            dpadupPressed = true;
        } else {
            if (dpadupPressed) {
                waitTime++;
                dpadupPressed = false;
            }
        }
        if (waitTime < 0) {
            dpaddownPressed = false;
            waitTime = 0;
        }
        if (gamepad1.y) {
            startingLocation = StartingLocation.RIGHT;
        } else if (gamepad1.a) {
            startingLocation = StartingLocation.LEFT;
        }

        if (gamepad1.x) {
            allianceColor = AllianceColor.BLUE;
        } else if (gamepad1.b) {
            allianceColor = AllianceColor.RED;
        }

        telemetry.addLine("Select Location and Alliance Color");
        telemetry.addData("Starting Location", startingLocation);
        telemetry.addData("Alliance Color", allianceColor);
        telemetry.addData("Time to Wait", waitTime);
    }

    @Override
    public void start() {
        robot.resetYaw();
        SparkFunOTOS.Pose2D position = robot.getSparkFunOTOS().getPosition();

        behaviors.add(new Wait(telemetry, waitTime * 1000));
        behaviors.add(new CloseClaw(telemetry, liftClaw));

        if (startingLocation == StartingLocation.LEFT) {
            behaviors.add(new DriveHeading(telemetry, robot, 0, 12, .2));
            behaviors.add(new TurnToHeading(telemetry, robot, 90, .006));
            behaviors.add(new DriveHeading(telemetry, robot, 90, 34, .5));
            behaviors.add(new TurnToHeading(telemetry, robot, 135, .006));
            behaviors.add(new RaiseLift(telemetry, robot, LiftHeight.HIGH));
            behaviors.add(new DriveHeading(telemetry, robot, 135, 15, .1));
            behaviors.add(new OpenClaw(telemetry, liftClaw));
            behaviors.add(new ReverseHeading(telemetry, robot, 135, -15, -0.2));
            behaviors.add(new LowerLift(telemetry, robot, LiftHeight.GROUND));
            behaviors.add(new TurnToHeading(telemetry, robot, 85, .006));
            //behaviors.add(new ReverseHeading(telemetry, robot, 85, -83, -0.75));
        } else {
            behaviors.add(new DriveHeading(telemetry, robot, 0, 6, .2));
            behaviors.add(new TurnToHeading(telemetry, robot, -90, .006));
            behaviors.add(new DriveHeading(telemetry, robot, -90, 48, .5));
        }
        behaviors.get(0).start();
    }

    @Override
    public void loop() {
        // checking to see if we still have behaviors left in the list
        if (state < behaviors.size()) {
            //checking to see if the current behavior is not done, run that behavior
            if (!behaviors.get(state).isDone()) {
                telemetry.addData("Running behavior", behaviors.get(state).getName());
                behaviors.get(state).run();
            } else {
                //increments the behavior
                state++;
                //starts next behavior if there are any left
                if (state < behaviors.size()) {
                    behaviors.get(state).start();
                }
            }
        } else {
            //if all behaviors are finished, stop the robot
            telemetry.addLine("Program done");
            this.stop();
        }


    }
}
