package org.firstinspires.ftc.teamcode.mechwarriors.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.mechwarriors.AllianceColor;
import org.firstinspires.ftc.teamcode.mechwarriors.JunctionType;
//import org.firstinspires.ftc.teamcode.mechwarriors.SignalSide;
import org.firstinspires.ftc.teamcode.mechwarriors.SignalSide;
import org.firstinspires.ftc.teamcode.mechwarriors.StartingLocation;
import org.firstinspires.ftc.teamcode.mechwarriors.behaviors.*;
import org.firstinspires.ftc.teamcode.mechwarriors.hardware.MechRobot;
import org.firstinspires.ftc.teamcode.mechwarriors.hardware.NominalSignalDetector;

import java.util.ArrayList;
import java.util.List;

@Autonomous(group = "MechWarriors")
public class PowerPlayAutoOpModeTesting extends OpMode {

    MechRobot robot;
    NominalSignalDetector signalDetector;

    AllianceColor allianceColor = AllianceColor.BLUE;
    StartingLocation startingLocation = StartingLocation.FRONT;
    SignalSide signalSide = SignalSide.NONE;

    List<Behavior> behaviors = new ArrayList<Behavior>();
    int state = 0;

    @Override
    public void init() {
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        robot = new MechRobot(hardwareMap);
        //robot.getClaw().close();
        signalDetector = new NominalSignalDetector(telemetry, hardwareMap);
        telemetry.addLine("Init done");
    }

    @Override
    public void init_loop() {
        signalSide = signalDetector.detect();

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

        behaviors.add(new CloseClaw(telemetry, robot.getClaw()));
        behaviors.add(new RaiseLift(telemetry, robot, JunctionType.TRAVEL));
        //behaviors.add(new DriveHeading(telemetry, "drive forward at 0", robot, 0, robot.calculateDriveTicks(24), 0.25));

        if (startingLocation == StartingLocation.FRONT && allianceColor == AllianceColor.BLUE) {
            behaviors.add(new Translate(telemetry, robot, 0, robot.calculateDriveTicks(44), -0.50));
            behaviors.add(new DriveHeading(telemetry, robot, 0, robot.calculateDriveTicks(47), 0.50));
            behaviors.add(new RaiseLift(telemetry, robot, JunctionType.HIGH));
            behaviors.add(new Translate(telemetry, robot, 0, robot.calculateDriveTicks(25), -0.30));
            behaviors.add(new OpenClaw(telemetry, robot.getClaw()));
            behaviors.add(new Translate(telemetry, robot, 0, robot.calculateDriveTicks(12), 0.30));
            //behaviors.add(new TurnToHeading(telemetry, "turn left to -90°", robot, -90));
            //behaviors.add(new DriveHeading(telemetry, "drive forward at 0°", robot, 0, robot.calculateDriveTicks(10), 0.25));
            //behaviors.add(new TurnToHeading(telemetry, "turn left to -45°", robot, -45));
            //behaviors.add(new DriveHeading(telemetry, "drive forward at -45°", robot, -45, robot.calculateDriveTicks(9), 0.1));
            behaviors.add(new LowerLift(telemetry, robot, JunctionType.TRAVEL));
            behaviors.add(new ReverseHeading(telemetry, robot, 0, -robot.calculateDriveTicks(47), -0.50));
            behaviors.add(new Translate(telemetry, robot, 0, robot.calculateDriveTicks(24), 0.50));


            switch (signalSide) {
                case ONE:
                    behaviors.add(new DriveHeading(telemetry, robot, -90, -robot.calculateDriveTicks(2), 0.25));
                    break;
                case TWO:
                    behaviors.add(new ReverseHeading(telemetry, robot, 90, -robot.calculateDriveTicks(22), -0.25));
                    break;
                case THREE:
                    behaviors.add(new ReverseHeading(telemetry, robot, 90, -robot.calculateDriveTicks(46), -0.25));
                    break;
                case NONE:
                    //behaviors.add(new ReverseHeading(telemetry, "drive backward", robot, 90, -robot.calculateDriveTicks(46), -0.25));
                    //behaviors.add(new TurnToHeading(telemetry, "turn left to 0°", robot, 0));
                    //behaviors.add(new ReverseHeading(telemetry, "drive backward", robot, 0, -robot.calculateDriveTicks(22), -0.25));
                    break;
            }

        } else if (startingLocation == StartingLocation.BACK && allianceColor == AllianceColor.BLUE) {

            behaviors.add(new TurnToHeading(telemetry, robot, 90));
            behaviors.add(new DriveHeading(telemetry, robot, 90, robot.calculateDriveTicks(20), 0.25));
            behaviors.add(new TurnToHeading(telemetry, robot, 45));
            behaviors.add(new RaiseLift(telemetry, robot, JunctionType.HIGH));
            behaviors.add(new DriveHeading(telemetry, robot, 45, robot.calculateDriveTicks(9), 0.1));
            behaviors.add(new OpenClaw(telemetry, robot.getClaw()));
            behaviors.add(new ReverseHeading(telemetry, robot, -45, -robot.calculateDriveTicks(8), -0.1));
            behaviors.add(new LowerLift(telemetry, robot, JunctionType.TRAVEL));
            behaviors.add(new TurnToHeading(telemetry, robot, 90));

            switch (signalSide) {
                case ONE:
                    behaviors.add(new ReverseHeading(telemetry, robot, -90, -robot.calculateDriveTicks(46), -0.25));
                    break;
                case TWO:
                    behaviors.add(new ReverseHeading(telemetry, robot, -90, -robot.calculateDriveTicks(22), -0.25));
                    break;
                case THREE:
                    behaviors.add(new DriveHeading(telemetry, robot, 90, -robot.calculateDriveTicks(2), 0.25));
                    break;
                case NONE:
                    behaviors.add(new ReverseHeading(telemetry, robot, -90, -robot.calculateDriveTicks(46), -0.25));
                    behaviors.add(new TurnToHeading(telemetry, robot, 0));
                    behaviors.add(new ReverseHeading(telemetry, robot, 0, -robot.calculateDriveTicks(22), -0.25));
                    break;
            }

        } else if (startingLocation == StartingLocation.FRONT && allianceColor == AllianceColor.RED) {

            behaviors.add(new TurnToHeading(telemetry, robot, 90));
            behaviors.add(new DriveHeading(telemetry, robot, 90, robot.calculateDriveTicks(20), 0.25));
            behaviors.add(new TurnToHeading(telemetry, robot, 45));
            behaviors.add(new RaiseLift(telemetry, robot, JunctionType.HIGH));
            behaviors.add(new DriveHeading(telemetry, robot, 45, robot.calculateDriveTicks(9), 0.1));
            behaviors.add(new OpenClaw(telemetry, robot.getClaw()));
            behaviors.add(new ReverseHeading(telemetry, robot, -45, -robot.calculateDriveTicks(8), -0.1));
            behaviors.add(new LowerLift(telemetry, robot, JunctionType.TRAVEL));
            behaviors.add(new TurnToHeading(telemetry, robot, 90));

            switch (signalSide) {
                case ONE:
                    behaviors.add(new ReverseHeading(telemetry, robot, -90, -robot.calculateDriveTicks(46), -0.25));
                    break;
                case TWO:
                    behaviors.add(new ReverseHeading(telemetry, robot, -90, -robot.calculateDriveTicks(22), -0.25));
                    break;
                case THREE:
                    behaviors.add(new DriveHeading(telemetry, robot, 90, -robot.calculateDriveTicks(2), 0.25));
                    break;
                case NONE:
                    behaviors.add(new ReverseHeading(telemetry, robot, -90, -robot.calculateDriveTicks(46), -0.25));
                    behaviors.add(new TurnToHeading(telemetry, robot, 0));
                    behaviors.add(new ReverseHeading(telemetry, robot, 0, -robot.calculateDriveTicks(22), -0.25));
                    break;
            }

        } else if (startingLocation == StartingLocation.BACK && allianceColor == AllianceColor.RED) {

            behaviors.add(new TurnToHeading(telemetry, robot, -90));
            behaviors.add(new DriveHeading(telemetry, robot, -90, robot.calculateDriveTicks(20), 0.25));
            behaviors.add(new TurnToHeading(telemetry, robot, -45));
            behaviors.add(new RaiseLift(telemetry, robot, JunctionType.HIGH));
            behaviors.add(new DriveHeading(telemetry, robot, -45, robot.calculateDriveTicks(9), 0.1));
            behaviors.add(new OpenClaw(telemetry, robot.getClaw()));
            behaviors.add(new ReverseHeading(telemetry, robot, 45, -robot.calculateDriveTicks(8), -0.1));
            behaviors.add(new LowerLift(telemetry, robot, JunctionType.TRAVEL));
            behaviors.add(new TurnToHeading(telemetry, robot, -90));

            switch (signalSide) {
                case ONE:
                    behaviors.add(new DriveHeading(telemetry, robot, -90, -robot.calculateDriveTicks(2), 0.25));
                    break;
                case TWO:
                    behaviors.add(new ReverseHeading(telemetry, robot, 90, -robot.calculateDriveTicks(22), -0.25));
                    break;
                case THREE:
                    behaviors.add(new ReverseHeading(telemetry, robot, 90, -robot.calculateDriveTicks(46), -0.25));
                    break;
                case NONE:
                    behaviors.add(new ReverseHeading(telemetry, robot, 90, -robot.calculateDriveTicks(46), -0.25));
                    behaviors.add(new TurnToHeading(telemetry, robot, 0));
                    behaviors.add(new ReverseHeading(telemetry, robot, 0, -robot.calculateDriveTicks(22), -0.25));
                    break;
            }
        }

        behaviors.add(new LowerLift(telemetry, robot, JunctionType.GROUND));

        behaviors.get(0).start();
    }

    @Override
    public void loop() {
        telemetry.addLine("Running program...");
        telemetry.addData("Starting Location", startingLocation);
        telemetry.addData("Alliance Color", allianceColor);
        telemetry.addData("Signal Side", signalSide);

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

