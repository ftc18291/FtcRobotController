package org.firstinspires.ftc.teamcode.mechwarriors.BioBuzz.Classes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechwarriors.BioBuzz.Classes.Enumerations.NServoState;
import org.firstinspires.ftc.teamcode.mechwarriors.BioBuzz.Classes.Enumerations.PNFlowerServoState;
import org.firstinspires.ftc.teamcode.mechwarriors.BioBuzz.Classes.Enumerations.PServoState;

@TeleOp
public class BioBuzzTeleOp extends OpMode {


    IntakeSpinner intakeSpinner;

    PLauncher pnLauncher;

    PServo pServo;
    NServo nServo;

    PNFlowerServo pnFlowerServo;
    private DriveTrain drivetrain;

    PServoState pServoState = PServoState.HOME;
    NServoState nServoState = NServoState.HOME;
    PNFlowerServoState pnFlowerServoState = PNFlowerServoState.IN;

    @Override
    public void init() {
        //Initialize hardware
        intakeSpinner = new IntakeSpinner(hardwareMap);
        pnLauncher = new PLauncher(hardwareMap, telemetry);
        drivetrain = new DriveTrain(hardwareMap, telemetry);
        pServo = new PServo(hardwareMap);
        nServo = new NServo(hardwareMap);
        pnFlowerServo = new PNFlowerServo(hardwareMap);
    }

    @Override
    public void loop() {
        //Intake Spinner
        if (gamepad2.dpad_up) {
            intakeSpinner.runIntakeSpinner();

        } else if (gamepad2.dpad_down) {
            intakeSpinner.reverseRunIntakeSpinner();
        } else {
            intakeSpinner.stopIntakeSpinner();
        }

       //PLauncher
        if (gamepad2.right_bumper) {
            pServo.setPositionLAUNCH();
        } else {pServo.setPositionHOME();}
       //NLauncher
        if (gamepad2.left_bumper) {
            nServo.setPositionLAUNCH();
        } else {nServo.setPositionHOME();}

        //PNFlowerServo
        if (gamepad2.x && PNFlowerServoState.IN== pnFlowerServoState) {
            pnFlowerServo.setPositionOUT();
        }
        if (gamepad2.x && PNFlowerServoState.OUT == pnFlowerServoState) {
            pnFlowerServo.setPositionIN();
        }
        //PServo
        if (gamepad2.right_trigger > 0.3 && PServoState.HOME == pServoState) {
            pServo.setPositionLAUNCH();
        } else  {pServo.setPositionHOME();}
        //NServo
        if (gamepad2.left_trigger > 0.3 && NServoState.HOME == nServoState) {
            nServo.setPositionLAUNCH();
        } else  {nServo.setPositionHOME();}


        // telemetry.addData("Intake Speed:", gamepad2.right_trigger);

        double y = gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;
            if (gamepad2.right_trigger > 0.3) {
            pnLauncher.startPNLauncher();
        }


        drivetrain.drive(x, y, rx);

    }
}
