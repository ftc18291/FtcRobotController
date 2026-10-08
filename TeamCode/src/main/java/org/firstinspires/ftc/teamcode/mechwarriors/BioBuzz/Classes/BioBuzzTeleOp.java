package org.firstinspires.ftc.teamcode.mechwarriors.BioBuzz.Classes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class BioBuzzTeleOp extends OpMode {


    IntakeSpinner intakeSpinner;

//private DriveTrain drivetrain;

    PNLauncher pnLauncher;

    PNServo pnServo;
    PNFlowerServo pnFlowerServo;
    private DriveTrain drivetrain;

    PnServoState pnServoState = PnServoState.HOME;
    PNFlowerServoState pnFlowerServoState = PNFlowerServoState.IN;

    @Override
    public void init() {
        intakeSpinner = new IntakeSpinner(hardwareMap);
        pnLauncher = new PNLauncher(hardwareMap, telemetry);
        drivetrain = new DriveTrain(hardwareMap, telemetry);
        pnServo = new PNServo(hardwareMap);
        pnFlowerServo = new PNFlowerServo(hardwareMap);
    }

    @Override
    public void loop() {
        if (gamepad2.dpad_up) {
            intakeSpinner.runIntakeSpinner();

        } else if (gamepad2.dpad_down) {
            intakeSpinner.reverseRunIntakeSpinner();
        } else {
            intakeSpinner.stopIntakeSpinner();
        }


        if (gamepad2.left_trigger > 0.5 ) {
            pnServo.setPositionLAUNCH();
        } else {pnServo.setPositionHOME();}

        if (gamepad2.x && PNFlowerServoState.IN== pnFlowerServoState) {
            pnFlowerServo.setPositionOUT();
        }
        if (gamepad2.x && PNFlowerServoState.OUT == pnFlowerServoState) {
            pnFlowerServo.setPositionIN();
        }



        // telemetry.addData("Intake Speed:", gamepad2.right_trigger);

        double y = gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;
        //    if (gamepad2.right_trigger > 0.3) {
//            pnLauncher.startPNLauncher(pnLauncherSpeed);
//        }


        drivetrain.drive(x, y, rx);

    }
}
