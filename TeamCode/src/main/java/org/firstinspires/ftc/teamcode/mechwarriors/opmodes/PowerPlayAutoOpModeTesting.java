package org.firstinspires.ftc.teamcode.mechwarriors.opmodes;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.mechwarriors.AllianceColor;
import org.firstinspires.ftc.teamcode.mechwarriors.JunctionType;
//import org.firstinspires.ftc.teamcode.mechwarriors.SignalSide;
import org.firstinspires.ftc.teamcode.mechwarriors.SignalSide;
import org.firstinspires.ftc.teamcode.mechwarriors.StartingLocation;
import org.firstinspires.ftc.teamcode.mechwarriors.behaviors.*;
import org.firstinspires.ftc.teamcode.mechwarriors.hardware.AprilTagSignalDetector;
import org.firstinspires.ftc.teamcode.mechwarriors.hardware.MechRobot;
import org.firstinspires.ftc.teamcode.mechwarriors.hardware.NominalSignalDetector;
import org.firstinspires.ftc.teamcode.mechwarriors.hardware.SignalDetector;

import java.util.ArrayList;
import java.util.List;

@Autonomous(group = "MechWarriors")
public class PowerPlayAutoOpModeTesting extends OpMode {

    MechRobot robot;
    SignalDetector signalDetector;

    AllianceColor allianceColor = AllianceColor.BLUE;
    StartingLocation startingLocation = StartingLocation.FRONT;
    SignalSide signalSide = SignalSide.NONE;


    List<Behavior> behaviors = new ArrayList<Behavior>();
    int state = 0;

    List<LynxModule> allHubs = new ArrayList<LynxModule>();

    @Override
    public void init() {
        allHubs = hardwareMap.getAll(LynxModule.class);
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        robot = new MechRobot(hardwareMap);
        signalDetector = new AprilTagSignalDetector(telemetry, hardwareMap);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

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

        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        telemetry.update();
    }

    @Override
    public void start() {

        behaviors.add(new CloseClaw(telemetry, robot.getClaw()));
        behaviors.add(new RaiseLift(telemetry, robot, JunctionType.TRAVEL));
        // behaviors.add(new ConcurrentActions(telemetry, new CloseClaw(telemetry, robot.getClaw()), new RaiseLift(telemetry, robot, JunctionType.TRAVEL )));
        //behaviors.add(new DriveHeading(telemetry, "drive forward at 0", robot, 0, robot.calculateDriveTicks(24), 0.25));

        if ((startingLocation == StartingLocation.BACK && allianceColor == AllianceColor.BLUE) || (startingLocation == StartingLocation.FRONT && allianceColor == AllianceColor.RED)) {
            behaviors.add(new DriveHeading(telemetry, robot, 0, robot.calculateDriveTicks(1), 0.30));
            behaviors.add(new Translate(telemetry, robot, 0, robot.calculateDriveTicks(24), -0.30));
            behaviors.add(new DriveHeading(telemetry, robot, 0, robot.calculateDriveTicks(22), 0.30));
            behaviors.add(new RaiseLift(telemetry, robot, JunctionType.HIGH));
            behaviors.add(new Translate(telemetry, robot, 0, robot.calculateDriveTicks(14.25), -0.30));
            //behaviors.add(new DriveHeading(telemetry, robot, 0, robot.calculateDriveTicks(2), 0.25));
            behaviors.add(new FindJunction(telemetry, robot, robot.calculateDriveTicks(3)));
            //behaviors.add(new LowerLift(telemetry, robot, JunctionType.MEDIUM));
            behaviors.add(new OpenClaw(telemetry, robot.getClaw()));
            behaviors.add(new ReverseHeading(telemetry, robot, 0, -robot.calculateDriveTicks(2), -0.25));
            behaviors.add(new Translate(telemetry, robot, 0, robot.calculateDriveTicks(15.25), 0.30));
            behaviors.add(new LowerLift(telemetry, robot, JunctionType.TRAVEL));

            switch (signalSide) {
                case ONE:
                    break;
                case TWO:
                    behaviors.add(new Translate(telemetry, robot, 0, robot.calculateDriveTicks(24), 0.50));
                    break;
                case THREE:
                    behaviors.add(new Translate(telemetry, robot, 0, robot.calculateDriveTicks(48), 0.50));
                    break;
                case NONE:
                    //behaviors.add(new ReverseHeading(telemetry, "drive backward", robot, 90, -robot.calculateDriveTicks(46), -0.25));
                    //behaviors.add(new TurnToHeading(telemetry, "turn left to 0°", robot, 0));
                    //behaviors.add(new ReverseHeading(telemetry, "drive backward", robot, 0, -robot.calculateDriveTicks(22), -0.25));
                    break;
            }

        } else {

            behaviors.add(new DriveHeading(telemetry, robot, 0, robot.calculateDriveTicks(1), 0.30));
            behaviors.add(new Translate(telemetry, robot, 0, robot.calculateDriveTicks(24), 0.30));
            behaviors.add(new DriveHeading(telemetry, robot, 0, robot.calculateDriveTicks(22), 0.30));
            behaviors.add(new RaiseLift(telemetry, robot, JunctionType.HIGH));
            behaviors.add(new Translate(telemetry, robot, 0, robot.calculateDriveTicks(14.5), 0.30));
            //behaviors.add(new DriveHeading(telemetry, robot, 0, robot.calculateDriveTicks(2), 0.25));
            behaviors.add(new FindJunction(telemetry, robot, robot.calculateDriveTicks(3)));
            //behaviors.add(new LowerLift(telemetry, robot, JunctionType.MEDIUM));
            behaviors.add(new OpenClaw(telemetry, robot.getClaw()));
            behaviors.add(new ReverseHeading(telemetry, robot, 0, -robot.calculateDriveTicks(2), -0.25));
            behaviors.add(new Translate(telemetry, robot, 0, robot.calculateDriveTicks(12), -0.30));
            behaviors.add(new LowerLift(telemetry, robot, JunctionType.TRAVEL));

            switch (signalSide) {
                case ONE:
                    behaviors.add(new Translate(telemetry, robot, 0, robot.calculateDriveTicks(48), -0.5));
                    break;
                case TWO:
                    behaviors.add(new Translate(telemetry, robot, 0, robot.calculateDriveTicks(24), -0.5));
                    break;
                case THREE:
                    break;
                case NONE:
                    behaviors.add(new ReverseHeading(telemetry, robot, -90, -robot.calculateDriveTicks(46), -0.25));
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

        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
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

