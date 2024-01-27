package org.firstinspires.ftc.teamcode.mechwarriors.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.mechwarriors.AllianceColor;
import org.firstinspires.ftc.teamcode.mechwarriors.JunctionType;
import org.firstinspires.ftc.teamcode.mechwarriors.SignalSide;
import org.firstinspires.ftc.teamcode.mechwarriors.StartingLocation;
import org.firstinspires.ftc.teamcode.mechwarriors.behaviors.*;
import org.firstinspires.ftc.teamcode.mechwarriors.hardware.MechRobot;

import java.util.ArrayList;
import java.util.List;

@Autonomous(group = "MechWarriors")
@Disabled
public class PowerPlayAutoOpMode extends OpMode {

    MechRobot robot;

    AllianceColor allianceColor = AllianceColor.BLUE;
    StartingLocation startingLocation = StartingLocation.LEFT;
    SignalSide signalSide = SignalSide.NONE;

    List<Behavior> behaviors = new ArrayList<>();
    int state = 0;

    @Override
    public void init() {
        robot = new MechRobot(hardwareMap);
        telemetry.addLine("Init done");
    }

    @Override
    public void init_loop() {
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
        telemetry.addData("Signal Sign", signalSide);
    }

    @Override
    public void start() {
        if (startingLocation == StartingLocation.LEFT && allianceColor == AllianceColor.BLUE) {
            if (signalSide == SignalSide.ONE) {

            }
            behaviors.add(new RaiseLift(telemetry, robot, JunctionType.MEDIUM));
            behaviors.add(new LowerLift(telemetry, robot, JunctionType.GROUND));
            behaviors.add(new OpenClaw(telemetry, robot.getLeftClaw()));
            behaviors.add(new CloseClaw(telemetry, robot.getLeftClaw()));
            behaviors.add(new OpenClaw(telemetry, robot.getLeftClaw()));
            behaviors.add(new TurnToHeading(telemetry, robot, 90, 1));
            behaviors.add(new DriveHeading(telemetry, robot, 90, 4000, 0.5));
        } else if (startingLocation == StartingLocation.RIGHT && allianceColor == AllianceColor.BLUE) {
            behaviors.add(new Translate(telemetry, robot, 0, 3000, 0.5));
            //behaviors.add(new TurnToHeading(telemetry, "turn left to -90", robot, -90));
            behaviors.add(new ReverseHeading(telemetry, robot, 0, -3000, -1.0));
        } else if (startingLocation == StartingLocation.LEFT && allianceColor == AllianceColor.RED) {
            behaviors.add(new TurnToHeading(telemetry, robot, -180, 1));
        } else if (startingLocation == StartingLocation.RIGHT && allianceColor == AllianceColor.RED) {
            behaviors.add(new TurnToHeading(telemetry, robot, 225, 1));
        }

        behaviors.get(0).start();
    }

    @Override
    public void loop() {
        telemetry.addLine("Running program...");
        telemetry.addData("Starting Location", startingLocation);
        telemetry.addData("Alliance Color", allianceColor);

        if (state < behaviors.size()) {
            if (!behaviors.get(state).isDone()) {
                telemetry.addData("Running behavior:", behaviors.get(state).getName());
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
}
