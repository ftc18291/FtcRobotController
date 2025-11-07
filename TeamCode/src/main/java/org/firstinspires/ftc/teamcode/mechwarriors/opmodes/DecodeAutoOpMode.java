package org.firstinspires.ftc.teamcode.mechwarriors.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechwarriors.AllianceColor;
import org.firstinspires.ftc.teamcode.mechwarriors.StartingLocation;
import org.firstinspires.ftc.teamcode.mechwarriors.behaviors.Behavior;
import org.firstinspires.ftc.teamcode.mechwarriors.behaviors.PedroPath;
import org.firstinspires.ftc.teamcode.mechwarriors.behaviors.RotateArtifactSorter;
import org.firstinspires.ftc.teamcode.mechwarriors.behaviors.ShootArtifact;
import org.firstinspires.ftc.teamcode.mechwarriors.behaviors.Wait;
import org.firstinspires.ftc.teamcode.mechwarriors.hardware.ArtifactIntaker;
import org.firstinspires.ftc.teamcode.mechwarriors.hardware.ArtifactSorter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;
import java.util.List;

@Config
@Autonomous(name = "Decode Auto OpMode")
public class DecodeAutoOpMode extends OpMode {


    private Follower follower;

    ArtifactIntaker artifactIntaker;

    ArtifactSorter artifactSorter;

    DcMotorEx launcherMotor;
    Servo launcherServo;

    Limelight3A limelight;


    List<Behavior> behaviors = new ArrayList<>();
    int state = 0;
    AllianceColor allianceColor = AllianceColor.BLUE;
    StartingLocation startingLocation = StartingLocation.LEFT;
    int waitTime = 0;
    boolean dpaddownPressed = false;
    boolean dpadupPressed = false;

    //Blue
    private final Pose blueStartPose = new Pose(38, 135.5, Math.toRadians(90));
    private final Pose blueObeliskPose = new Pose(57.3, 87.4, Math.toRadians(80));
    private final Pose blueScorePose = new Pose(37.7, 108.3, Math.toRadians(135));
    private final Pose blueLeavePose = new Pose(40.5, 61.5, Math.toRadians(180));

    //Red
    private final Pose redStartPose = new Pose(104.6, 135.5, Math.toRadians(90));
    private final Pose redObeliskPose = new Pose(86.8, 87.4, Math.toRadians(100));
    private final Pose redScorePose = new Pose(110.8, 108.3, Math.toRadians(35));
    private final Pose redLeavePose = new Pose(102.4, 61.5, Math.toRadians(0));


    private Path goToBlueObelisk;
    private Path gotoBlueScore;
    private Path goToBlueLeave;


    private Path goToRedObelisk;
    private Path goToRedScorePose;
    private Path goToRedLeave;


    @Override
    public void init() {
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1);

        artifactIntaker = new ArtifactIntaker(hardwareMap);
        artifactIntaker.setSweeperToRearPosition();

        artifactSorter = new ArtifactSorter(hardwareMap);

        launcherMotor = hardwareMap.get(DcMotorEx.class, "launcherMotor");
        launcherMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        launcherServo = hardwareMap.get(Servo.class, "launcherServo");
        launcherServo.scaleRange(0.65, 1.0);
        launcherServo.setPosition(0);

        follower = Constants.createFollower(hardwareMap);

        buildPaths();

        limelight.start();

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
        artifactIntaker.setSweeperToRearPosition();
        behaviors.add(new Wait(telemetry, waitTime * 1000));

        if (allianceColor == AllianceColor.BLUE) {
            follower.setStartingPose(blueStartPose);

            // Drive to score position
            behaviors.add(new PedroPath(follower, goToBlueObelisk, blueObeliskPose, telemetry));
            behaviors.add(new Wait(telemetry, 1000));
            behaviors.add(new PedroPath(follower, gotoBlueScore, blueScorePose, telemetry));

            // Shoot three artifacts
            behaviors.add(new ShootArtifact(telemetry, launcherMotor, launcherServo, 2200));
            behaviors.add(new RotateArtifactSorter(telemetry, artifactSorter));
            behaviors.add(new ShootArtifact(telemetry, launcherMotor, launcherServo, 2200));
            behaviors.add(new RotateArtifactSorter(telemetry, artifactSorter));
            behaviors.add(new ShootArtifact(telemetry, launcherMotor, launcherServo, 2200));

            // Drive to park position
            behaviors.add(new PedroPath(follower, goToBlueLeave, blueLeavePose, telemetry));
        } else {
            follower.setStartingPose(redStartPose);

            //Drive to score position
            behaviors.add(new PedroPath(follower, goToRedObelisk, redObeliskPose, telemetry));
            behaviors.add(new Wait(telemetry, 1000));
            behaviors.add(new PedroPath(follower, goToRedScorePose, redScorePose, telemetry));

            //Shoot three artifacts
            behaviors.add(new ShootArtifact(telemetry, launcherMotor, launcherServo, 2200));
            behaviors.add(new RotateArtifactSorter(telemetry, artifactSorter));
            behaviors.add(new ShootArtifact(telemetry, launcherMotor, launcherServo, 2200));
            behaviors.add(new RotateArtifactSorter(telemetry, artifactSorter));
            behaviors.add(new ShootArtifact(telemetry, launcherMotor, launcherServo, 2200));

            //Drive to park position
            behaviors.add(new PedroPath(follower, goToRedLeave, redLeavePose, telemetry));

        }

        if (startingLocation == StartingLocation.LEFT) {

        } else {

        }
        artifactSorter.init();
        //  behaviors.get(0).start();
    }

    @Override
    public void loop() {
        follower.update();
        runBehaviors();


        telemetry.addData("state", state);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();

        follower.update();
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
        // Blue
        goToBlueObelisk = new Path(new BezierLine(blueStartPose, blueObeliskPose));
        goToBlueObelisk.setLinearHeadingInterpolation(blueStartPose.getHeading(), blueObeliskPose.getHeading());

        gotoBlueScore = new Path((new BezierLine(blueObeliskPose, blueScorePose)));
        gotoBlueScore.setLinearHeadingInterpolation(blueObeliskPose.getHeading(), blueScorePose.getHeading());

        goToBlueLeave = new Path(new BezierLine(blueScorePose, blueLeavePose));
        goToBlueLeave.setLinearHeadingInterpolation(blueScorePose.getHeading(), blueLeavePose.getHeading());

        // Red
        goToRedObelisk = new Path(new BezierLine(redStartPose, redObeliskPose));
        goToRedObelisk.setLinearHeadingInterpolation(redStartPose.getHeading(), redObeliskPose.getHeading());

        goToRedScorePose = new Path(new BezierLine(redObeliskPose, redScorePose));
        goToRedScorePose.setLinearHeadingInterpolation(redObeliskPose.getHeading(), redScorePose.getHeading());

        goToRedLeave = new Path(new BezierLine(redScorePose, redLeavePose));
        goToRedLeave.setLinearHeadingInterpolation(redScorePose.getHeading(), redLeavePose.getHeading());
    }
}
