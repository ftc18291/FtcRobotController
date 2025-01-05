package org.firstinspires.ftc.teamcode.mechwarriors.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechwarriors.AllianceColor;
import org.firstinspires.ftc.teamcode.mechwarriors.ClawArmPosition;
import org.firstinspires.ftc.teamcode.mechwarriors.LiftHeight;
import org.firstinspires.ftc.teamcode.mechwarriors.StartingLocation;
import org.firstinspires.ftc.teamcode.mechwarriors.behaviors.Behavior;
import org.firstinspires.ftc.teamcode.mechwarriors.behaviors.CloseClaw;
import org.firstinspires.ftc.teamcode.mechwarriors.behaviors.ConcurrentActions;
import org.firstinspires.ftc.teamcode.mechwarriors.behaviors.OpenClaw;
import org.firstinspires.ftc.teamcode.mechwarriors.behaviors.PedroPath;
import org.firstinspires.ftc.teamcode.mechwarriors.behaviors.SetClawArmPosition;
import org.firstinspires.ftc.teamcode.mechwarriors.behaviors.SetLiftHeight;
import org.firstinspires.ftc.teamcode.mechwarriors.behaviors.Wait;
import org.firstinspires.ftc.teamcode.mechwarriors.hardware.Claw;
import org.firstinspires.ftc.teamcode.mechwarriors.hardware.ClawArmPID;
import org.firstinspires.ftc.teamcode.mechwarriors.hardware.LinearSlideLiftPID;
import org.firstinspires.ftc.teamcode.mechwarriors.hardware.SampleClaw;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

import java.util.ArrayList;
import java.util.List;

@Autonomous(group = "IntoTheDeep", name = "Pedro Auto OpMode")
public class IntoTheDeepPedroPathAutoOpMode extends OpMode {

    //Robot robot;
    LinearSlideLiftPID lift;
    ClawArmPID clawArm; // 22.5 inches from center of robot to fixed claw
    Claw sampleClaw;

    private Follower follower;

    List<Behavior> behaviors = new ArrayList<>();
    int state = 0;
    AllianceColor allianceColor = AllianceColor.BLUE;
    StartingLocation startingLocation = StartingLocation.LEFT;
    int waitTime = 0;
    boolean dpaddownPressed = false;
    boolean dpadupPressed = false;

    private final Pose startPose = new Pose(9, 104.5, Math.toRadians(90));
    private final Pose scorePose = new Pose(14, 129, Math.toRadians(135));
    private final Pose pickup1Pose = new Pose(31, 107, Math.toRadians(45));
    private final Pose pickup2Pose = new Pose(31, 120, Math.toRadians(45));
    private final Pose pickup3Pose = new Pose(40, 124, Math.toRadians(75));

    private Path scorePreload;
    private Path goToSample1, scoreSample1;
    private Path goToSample2, scoreSample2;
    private Path goToSample3, scoreSample3;

    @Override
    public void init() {
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        // robot = new IntoTheDeepRobot(hardwareMap);
        lift = new LinearSlideLiftPID(hardwareMap);
        clawArm = new ClawArmPID(hardwareMap);

        sampleClaw = new SampleClaw(
                hardwareMap,
                "clawServo",
                true,
                0.25,
                0.55);
        //sampleClaw.close();

        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
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
        behaviors.add(new CloseClaw(telemetry, sampleClaw));
        behaviors.add(new Wait(telemetry, waitTime * 1000));

        if (startingLocation == StartingLocation.LEFT) {
            // Drive to scoring position
            behaviors.add(new ConcurrentActions(telemetry,
                    new PedroPath(follower, scorePreload, scorePose, telemetry),
                    new SetLiftHeight(telemetry, lift, LiftHeight.HIGH)
            ));

            // Score sample
            behaviors.add(new SetClawArmPosition(telemetry, clawArm, ClawArmPosition.SCORE));
            behaviors.add(new OpenClaw(telemetry, sampleClaw));
            behaviors.add(new SetClawArmPosition(telemetry, clawArm, ClawArmPosition.STOW));

            // Drive to sample 1
            behaviors.add(new ConcurrentActions(telemetry,
                    new PedroPath(follower, goToSample1, pickup1Pose, telemetry),
                    new SetLiftHeight(telemetry, lift, LiftHeight.RETRIEVE)
            ));

            // Pickup sample 1
            behaviors.add(new SetClawArmPosition(telemetry, clawArm, ClawArmPosition.PICKUP));
            behaviors.add(new CloseClaw(telemetry, sampleClaw));
            behaviors.add(new SetClawArmPosition(telemetry, clawArm, ClawArmPosition.STOW));

            // Drive to net
            behaviors.add(new ConcurrentActions(telemetry,
                    new PedroPath(follower, scoreSample1, scorePose, telemetry),
                    new SetLiftHeight(telemetry, lift, LiftHeight.HIGH)
            ));

            // Score sample
            behaviors.add(new SetClawArmPosition(telemetry, clawArm, ClawArmPosition.SCORE));
            behaviors.add(new OpenClaw(telemetry, sampleClaw));
            behaviors.add(new SetClawArmPosition(telemetry, clawArm, ClawArmPosition.STOW));

            // Drive to sample 2
            behaviors.add(new ConcurrentActions(telemetry,
                    new PedroPath(follower, goToSample2, pickup2Pose, telemetry),
                    new SetLiftHeight(telemetry, lift, LiftHeight.RETRIEVE)
            ));

            // Pickup sample 2
            behaviors.add(new SetClawArmPosition(telemetry, clawArm, ClawArmPosition.PICKUP));
            behaviors.add(new CloseClaw(telemetry, sampleClaw));
            behaviors.add(new SetClawArmPosition(telemetry, clawArm, ClawArmPosition.STOW));

            // Drive to net
            behaviors.add(new ConcurrentActions(telemetry,
                    new PedroPath(follower, scoreSample2, scorePose, telemetry),
                    new SetLiftHeight(telemetry, lift, LiftHeight.HIGH)
            ));

            // Score sample
            behaviors.add(new SetClawArmPosition(telemetry, clawArm, ClawArmPosition.SCORE));
            behaviors.add(new OpenClaw(telemetry, sampleClaw));
            behaviors.add(new SetClawArmPosition(telemetry, clawArm, ClawArmPosition.STOW));

            // Drive to sample 3
            behaviors.add(new ConcurrentActions(telemetry,
                    new PedroPath(follower, goToSample3, pickup3Pose, telemetry),
                    new SetLiftHeight(telemetry, lift, LiftHeight.RETRIEVE)
            ));

            // Pickup sample 3
            behaviors.add(new SetClawArmPosition(telemetry, clawArm, ClawArmPosition.PICKUP));
            behaviors.add(new CloseClaw(telemetry, sampleClaw));
            behaviors.add(new SetClawArmPosition(telemetry, clawArm, ClawArmPosition.STOW));

            // Drive to net
            behaviors.add(new ConcurrentActions(telemetry,
                    new PedroPath(follower, scoreSample3, scorePose, telemetry),
                    new SetLiftHeight(telemetry, lift, LiftHeight.HIGH)
            ));

            // Score sample
            behaviors.add(new SetClawArmPosition(telemetry, clawArm, ClawArmPosition.SCORE));
            behaviors.add(new OpenClaw(telemetry, sampleClaw));
            behaviors.add(new SetClawArmPosition(telemetry, clawArm, ClawArmPosition.STOW));

            // Lower lift
            behaviors.add(new SetLiftHeight(telemetry, lift, LiftHeight.BOTTOM));
        } else {
//            behaviors.add(new DriveHeading(telemetry, robot, 0, 6, .2));
//            behaviors.add(new TurnToHeading(telemetry, robot, -90, .006));
//            behaviors.add(new DriveHeading(telemetry, robot, -90, 48, .5));
        }
        behaviors.get(0).start();
    }

    @Override
    public void loop() {
        runBehaviors();

        lift.maintain();
        clawArm.maintain();

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
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePose)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        goToSample1 = new Path(new BezierLine(new Point(scorePose), new Point(pickup1Pose)));
        goToSample1.setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading());

        scoreSample1 = new Path(new BezierLine(new Point(pickup1Pose), new Point(scorePose)));
        scoreSample1.setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading());

        goToSample2 = new Path(new BezierLine(new Point(scorePose), new Point(pickup2Pose)));
        goToSample2.setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading());

        scoreSample2 = new Path(new BezierLine(new Point(pickup2Pose), new Point(scorePose)));
        scoreSample2.setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading());

        goToSample3 = new Path(new BezierLine(new Point(scorePose), new Point(pickup3Pose)));
        goToSample3.setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading());

        scoreSample3 = new Path(new BezierLine(new Point(pickup3Pose), new Point(scorePose)));
        scoreSample3.setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading());
    }
}
