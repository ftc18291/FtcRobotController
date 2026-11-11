package org.firstinspires.ftc.teamcode.mechwarriors.BioBuzz.Classes;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;



import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PLauncher {
    DcMotorEx pLauncher;
    Telemetry telemetry;
    PServo pServo;


    public void setpLauncherSpeed(int pLauncherSpeed) {
        this.pLauncherSpeed = pLauncherSpeed;
    }

    public int pLauncherSpeed = 1225;


    public PLauncher(HardwareMap hardwareMap, Telemetry telemetry, PServo pServo) {
        this.telemetry = telemetry;
        this.pServo = pServo;
        pLauncher = hardwareMap.get(DcMotorEx.class, "pLauncher");
        pLauncher.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        pLauncher.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        //pLauncher.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void startPLauncher() {
        pLauncher.setVelocity(pLauncherSpeed);
    }
    public void stopPLauncher() {
        pLauncher.setVelocity(0);
    }
    public double getVelocity() {
        return pLauncher.getVelocity();

    }
    public void shoot() {
        startPLauncher();
       if (getVelocity() > 1215 && getVelocity() < 1235) {
           pServo.setPositionLAUNCH();
       }

    }

}
