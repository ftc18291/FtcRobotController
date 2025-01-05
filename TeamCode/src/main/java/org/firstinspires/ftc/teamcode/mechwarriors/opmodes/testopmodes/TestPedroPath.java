package org.firstinspires.ftc.teamcode.mechwarriors.opmodes.testopmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechwarriors.AllianceColor;
import org.firstinspires.ftc.teamcode.mechwarriors.StartingLocation;
import org.firstinspires.ftc.teamcode.mechwarriors.behaviors.Behavior;
import org.firstinspires.ftc.teamcode.mechwarriors.behaviors.PedroPath;
import org.firstinspires.ftc.teamcode.mechwarriors.behaviors.Wait;
import org.firstinspires.ftc.teamcode.mechwarriors.hardware.Claw;
import org.firstinspires.ftc.teamcode.mechwarriors.hardware.ClawArmPID;
import org.firstinspires.ftc.teamcode.mechwarriors.hardware.IntoTheDeepRobot;
import org.firstinspires.ftc.teamcode.mechwarriors.hardware.LinearSlideLiftPID;
import org.firstinspires.ftc.teamcode.mechwarriors.hardware.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

import java.util.ArrayList;
import java.util.List;

@Disabled
@Autonomous(group = "IntoTheDeep", name = "Test Pedro Auto OpMode")
public class TestPedroPath extends OpMode {

    Robot robot;
    LinearSlideLiftPID lift;
    ClawArmPID clawArm;
    Claw liftClaw;

    private Follower follower;

    List<Behavior> behaviors = new ArrayList<>();
    int state = 0;
    AllianceColor allianceColor = AllianceColor.BLUE;
    StartingLocation startingLocation = StartingLocation.LEFT;
    int waitTime = 0;
    boolean dpaddownPressed = false;
    boolean dpadupPressed = false;

    private final Pose pose1 = new Pose(0, 0, Math.toRadians(90));
    private final Pose pose2 = new Pose(0, 36, Math.toRadians(90));

    private Path pose1ToPose2;
    private Path pose2ToPose1;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        robot = new IntoTheDeepRobot(hardwareMap);
        lift = new LinearSlideLiftPID(hardwareMap);
        clawArm = new ClawArmPID(hardwareMap);

        liftClaw = robot.getSampleClaw();
        liftClaw.close();

        follower = new Follower(hardwareMap);
        follower.setStartingPose(pose1);
        buildPaths();
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
        behaviors.add(new Wait(telemetry, waitTime * 1000));

        behaviors.add(new PedroPath(follower, pose1ToPose2, pose2, telemetry));
        behaviors.add(new PedroPath(follower, pose2ToPose1, pose1, telemetry));
        behaviors.add(new PedroPath(follower, pose1ToPose2, pose2, telemetry));
        behaviors.add(new PedroPath(follower, pose2ToPose1, pose1, telemetry));
        behaviors.add(new PedroPath(follower, pose1ToPose2, pose2, telemetry));
        behaviors.add(new PedroPath(follower, pose2ToPose1, pose1, telemetry));

        behaviors.get(0).start();
    }

    @Override
    public void loop() {
        runBehaviors();

        telemetry.addData("Lift position", lift.getHeight());
        telemetry.addData("Claw Arm position", clawArm.getPosition());

        telemetry.addData("state", state);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();

    }

    private void runBehaviors() {
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

    private void buildPaths() {
        pose1ToPose2 = new Path(new BezierLine(new Point(pose1), new Point(pose2)));
        pose1ToPose2.setLinearHeadingInterpolation(pose1.getHeading(), pose2.getHeading());

        pose2ToPose1 = new Path(new BezierLine(new Point(pose2), new Point(pose1)));
        pose2ToPose1.setLinearHeadingInterpolation(pose2.getHeading(), pose1.getHeading());

    }
}
