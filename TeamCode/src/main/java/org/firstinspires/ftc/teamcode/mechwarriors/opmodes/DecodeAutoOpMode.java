package org.firstinspires.ftc.teamcode.mechwarriors.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechwarriors.AllianceColor;
import org.firstinspires.ftc.teamcode.mechwarriors.StartingLocation;
import org.firstinspires.ftc.teamcode.mechwarriors.behaviors.Behavior;
import org.firstinspires.ftc.teamcode.mechwarriors.behaviors.PedroPath;
import org.firstinspires.ftc.teamcode.mechwarriors.behaviors.ReadObelisk;
import org.firstinspires.ftc.teamcode.mechwarriors.behaviors.RotateArtifactSorterOneSlot;
import org.firstinspires.ftc.teamcode.mechwarriors.behaviors.SetSweeperToFrontPosition;
import org.firstinspires.ftc.teamcode.mechwarriors.behaviors.SetSweeperToRearPosition;
import org.firstinspires.ftc.teamcode.mechwarriors.behaviors.ShootArtifact;
import org.firstinspires.ftc.teamcode.mechwarriors.behaviors.TurnOnIntake;
import org.firstinspires.ftc.teamcode.mechwarriors.behaviors.Wait;
import org.firstinspires.ftc.teamcode.mechwarriors.hardware.ArtifactIntaker;
import org.firstinspires.ftc.teamcode.mechwarriors.hardware.ArtifactLauncher;
import org.firstinspires.ftc.teamcode.mechwarriors.hardware.ArtifactSorter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.concurrent.atomic.AtomicInteger;

@Config
@Autonomous(name = "Decode Auto OpMode")
public class DecodeAutoOpMode extends OpMode {


    private Follower follower;

    ArtifactIntaker artifactIntaker;

    ArtifactSorter artifactSorter;

    ArtifactLauncher artifactLauncher;

    Limelight3A limelight;

    // Default to April Tag 21
    AtomicInteger obeliskId = new AtomicInteger(21);

    List<Behavior> behaviors = new ArrayList<>();
    int state = 0;
    AllianceColor allianceColor = AllianceColor.BLUE;
    StartingLocation startingLocation = StartingLocation.LEFT;
    int waitTime = 0;
    boolean dpaddownPressed = false;
    boolean dpadupPressed = false;

    //Blue
    private final Pose blueStartPose = new Pose(21.7, 122.9, Math.toRadians(53.5));
    //private final Pose blueStartPose = new Pose(33, 134.5, Math.toRadians(0));
    private final Pose blueObeliskPose = new Pose(57.3, 87.4, Math.toRadians(80));
    private final Pose blueScorePose = new Pose(40, 106, Math.toRadians(135));
    private final Pose blueLeavePose = new Pose(40, 60, Math.toRadians(180));
    private final Pose blueIntakePose = new Pose(106.5, 83.5, Math.toRadians(35));

    //Red
    private final Pose redStartPose = new Pose(122.3, 122.9, Math.toRadians(126.5));
    //private final Pose redStartPose = new Pose(111, 134.5, Math.toRadians(180));
    private final Pose redObeliskPose = new Pose(86.7, 87.4, Math.toRadians(100));
    private final Pose redScorePose = new Pose(104, 106, Math.toRadians(35));
    private final Pose redLeavePose = new Pose(103, 60, Math.toRadians(0));
    private final Pose redIntakePose = new Pose(39.7, 83.7);


    private Path goToBlueObelisk;
    private Path gotoBlueScore;
    private Path goToBlueLeave;
    private Path goToBlueIntake;


    private Path goToRedObelisk;
    private Path goToRedScorePose;
    private Path goToRedLeave;
    private Path goToRedIntake;


    @Override
    public void init() {
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1);

        artifactIntaker = new ArtifactIntaker(hardwareMap);
        artifactIntaker.setSweeperToRearPosition();
        artifactSorter = new ArtifactSorter(hardwareMap, telemetry);
        artifactLauncher = new ArtifactLauncher(hardwareMap, telemetry);

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
        behaviors.add(new SetSweeperToRearPosition(telemetry, artifactIntaker));
        behaviors.add(new Wait(telemetry, waitTime * 1000));

        if (allianceColor == AllianceColor.BLUE) {
            follower.setStartingPose(blueStartPose);

            // Drive to score position
            behaviors.add(new PedroPath(follower, goToBlueObelisk, blueObeliskPose, telemetry));
            behaviors.add(new ReadObelisk(limelight, telemetry, obeliskId));
            behaviors.add(new PedroPath(follower, gotoBlueScore, blueScorePose, telemetry));
            behaviors.add(new Wait(telemetry, 1000));

            // Shooting pattern gets added in loop

            // Drive to park position
            behaviors.add(new SetSweeperToFrontPosition(telemetry, artifactIntaker));
            behaviors.add(new PedroPath(follower, goToBlueLeave, blueLeavePose, telemetry));
        } else {
            follower.setStartingPose(redStartPose);

            //Drive to score position
            behaviors.add(new PedroPath(follower, goToRedObelisk, redObeliskPose, telemetry));
            behaviors.add(new ReadObelisk(limelight, telemetry, obeliskId));
            behaviors.add(new PedroPath(follower, goToRedScorePose, redScorePose, telemetry));
            behaviors.add(new Wait(telemetry, 1000));

            // Shooting pattern gets added in loop

            // Drive to park position
            behaviors.add(new SetSweeperToFrontPosition(telemetry, artifactIntaker));
            behaviors.add(new PedroPath(follower, goToRedLeave, redLeavePose, telemetry));
        }


        if (startingLocation == StartingLocation.LEFT) {

        } else {

        }
        artifactSorter.init();
    }

    @Override
    public void loop() {
        follower.update();
        telemetry.addData("sorter motor", artifactSorter.sorterMotor.getCurrentPosition());
        telemetry.addData("currentTicksTarget", artifactSorter.currentTicksTarget);
        runBehaviors();

        telemetry.addData("obeliskId", obeliskId);
        telemetry.addData("state", state);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();

        follower.update();
    }

    @Override
    public void stop() {
        limelight.stop();
        //limelight.shutdown();
    }

    private void runBehaviors() {
        // checking to see if we still have behaviors left in the list
        if (state < behaviors.size()) {
            //checking to see if the current behavior is not done, run that behavior
            if (!behaviors.get(state).isDone()) {
                telemetry.addData("Running behavior", behaviors.get(state).getName());
                behaviors.get(state).run();
            } else {
                if (Objects.nonNull(behaviors.get(state).getName()) &&
                        behaviors.get(state).getName().equals("ReadObelisk")) {
                    telemetry.addLine("Adding shooting behaviors");
                    behaviors.addAll(state + 2, buildShooterOrder());
                }
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

        goToBlueIntake = new Path(new BezierLine(blueScorePose, blueIntakePose));
        goToBlueIntake.setLinearHeadingInterpolation(blueScorePose.getHeading(), blueIntakePose.getHeading());

        goToBlueLeave = new Path(new BezierCurve(blueIntakePose, new Pose(62, 92), blueLeavePose));
        goToBlueLeave.setLinearHeadingInterpolation(blueIntakePose.getHeading(), blueLeavePose.getHeading());


        // Red
        goToRedObelisk = new Path(new BezierLine(redStartPose, redObeliskPose));
        goToRedObelisk.setLinearHeadingInterpolation(redStartPose.getHeading(), redObeliskPose.getHeading());

        goToRedScorePose = new Path(new BezierLine(redObeliskPose, redScorePose));
        goToRedScorePose.setLinearHeadingInterpolation(redObeliskPose.getHeading(), redScorePose.getHeading());

        goToRedIntake = new Path(new BezierLine(redIntakePose, blueLeavePose));
        goToRedIntake.setLinearHeadingInterpolation(redIntakePose.getHeading(), blueLeavePose.getHeading());

        goToRedLeave = new Path(new BezierCurve(redScorePose, new Pose(82, 92), redLeavePose));
        goToRedLeave.setLinearHeadingInterpolation(redScorePose.getHeading(), redLeavePose.getHeading());

    }

    private List<Behavior> buildShooterOrder() {
        List<Behavior> obeliskBehavior = new ArrayList<>();

        if (obeliskId.get() == 21) {
            obeliskBehavior.add(new ShootArtifact(telemetry, artifactLauncher));
            obeliskBehavior.add(new RotateArtifactSorterOneSlot(telemetry, artifactSorter));
            obeliskBehavior.add(new ShootArtifact(telemetry, artifactLauncher));
            obeliskBehavior.add(new RotateArtifactSorterOneSlot(telemetry, artifactSorter));
            obeliskBehavior.add(new ShootArtifact(telemetry, artifactLauncher));
        } else if (obeliskId.get() == 22) {
            obeliskBehavior.add(new RotateArtifactSorterOneSlot(telemetry, artifactSorter));
            obeliskBehavior.add(new ShootArtifact(telemetry, artifactLauncher));
            obeliskBehavior.add(new RotateArtifactSorterOneSlot(telemetry, artifactSorter));
            obeliskBehavior.add(new RotateArtifactSorterOneSlot(telemetry, artifactSorter));
            obeliskBehavior.add(new ShootArtifact(telemetry, artifactLauncher));
            obeliskBehavior.add(new RotateArtifactSorterOneSlot(telemetry, artifactSorter));
            obeliskBehavior.add(new RotateArtifactSorterOneSlot(telemetry, artifactSorter));
            obeliskBehavior.add(new ShootArtifact(telemetry, artifactLauncher));
        } else if (obeliskId.get() == 23) {
            obeliskBehavior.add(new RotateArtifactSorterOneSlot(telemetry, artifactSorter));
            obeliskBehavior.add(new ShootArtifact(telemetry, artifactLauncher));
            obeliskBehavior.add(new RotateArtifactSorterOneSlot(telemetry, artifactSorter));
            obeliskBehavior.add(new ShootArtifact(telemetry, artifactLauncher));
            obeliskBehavior.add(new RotateArtifactSorterOneSlot(telemetry, artifactSorter));
            obeliskBehavior.add(new ShootArtifact(telemetry, artifactLauncher));
        }

        return obeliskBehavior;
    }
}
