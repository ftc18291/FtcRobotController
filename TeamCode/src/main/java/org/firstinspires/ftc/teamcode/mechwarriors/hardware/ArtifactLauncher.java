package org.firstinspires.ftc.teamcode.mechwarriors.hardware;

import android.sax.StartElementListener;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class ArtifactLauncher {

    private static final double SLOW_SPEED = 2000.0;
    private static final double FAST_SPEED = 2400.0;

    DcMotorEx launcherMotor;
    Servo launcherServo;
    ServoImplEx leftLifterServo;
    ServoImplEx rightLifterServo;

    double rampPosition = 0.0;
    LEDIndicator ledIndicator;
    ShootingMode shootingMode = ShootingMode.SHORT;
    Double launcherMotorSpeed;

    PIDFCoefficients launchMotorPIDFValues;

    Telemetry telemetry;

    //public static double NEW_P = 10;
    //public static double NEW_I = 3;
    //public static double NEW_D = 0;
    //public static double NEW_F = 0;

    public static double NEW_P = 100;
    public static double NEW_I = 40;
    public static double NEW_D = 10;
    public static double NEW_F = 0;




    public ArtifactLauncher(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        launcherMotor = hardwareMap.get(DcMotorEx.class, "launcherMotor");
        launcherMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        launcherMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        launcherMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        launcherServo = hardwareMap.get(Servo.class, "launcherServo");
        launcherServo.scaleRange(0.65, 1.0);
        launcherServo.setDirection(Servo.Direction.REVERSE);
        launcherServo.setPosition(0);

        leftLifterServo = hardwareMap.get(ServoImplEx.class, "leftLifterServo");
        leftLifterServo.setPwmRange(new PwmControl.PwmRange(900, 2100));
        rightLifterServo = hardwareMap.get(ServoImplEx.class, "rightLifterServo");
        rightLifterServo.setPwmRange(new PwmControl.PwmRange(900, 2100));

        rightLifterServo.setPosition(0);
        leftLifterServo.setPosition(0);

        launcherMotorSpeed = SLOW_SPEED;

        ledIndicator = new LEDIndicator(hardwareMap, "launchSpeedIndicator");
        ledIndicator.setColor(LEDIndicator.LEDColor.OFF);

        launchMotorPIDFValues = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);
        launcherMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, launchMotorPIDFValues);
    }

    public void stopAll() {
        launcherMotor.setVelocity(0);
        ledIndicator.setColor(LEDIndicator.LEDColor.OFF);
    }

    public void startFlywheel() {
       /* telemetry.addData("P", launchMotorPIDFValues.p);
        telemetry.addData("I", launchMotorPIDFValues.i);
        telemetry.addData("D", launchMotorPIDFValues.d);
        telemetry.addData("F", launchMotorPIDFValues.f);


        launchMotorPIDFValues.p = NEW_P;
        launchMotorPIDFValues.i = NEW_I;
        launchMotorPIDFValues.d = NEW_D;
        launchMotorPIDFValues.f = NEW_F;
        launcherMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, launchMotorPIDFValues);*/
        launcherMotor.setVelocity(launcherMotorSpeed);
    }

    public void checkSpeed() {
        if (launcherMotor.getVelocity() == launcherMotorSpeed) {
            ledIndicator.setColor(LEDIndicator.LEDColor.GREEN);
        } else {
            ledIndicator.setColor(LEDIndicator.LEDColor.OFF);
        }
//        if (launcherMotor.getVelocity() > 0 && launcherMotor.getVelocity() < launcherMotorSpeed) {
//            ledIndicator.setColor(LEDIndicator.LEDColor.RED);
//        } else if (launcherMotor.getVelocity() >= launcherMotorSpeed &&
//                launcherMotor.getVelocity() <= launcherMotorSpeed + 10) {
//            ledIndicator.setColor(LEDIndicator.LEDColor.GREEN);
//        } else {
//            ledIndicator.setColor(LEDIndicator.LEDColor.OFF);
//        }
    }

    public void setRampPosition(double pos) {
        rampPosition = pos;
    }

    public void setShootingMode() {
        if (this.shootingMode == ShootingMode.SHORT) {
            shootingMode = ShootingMode.LONG;
        } else {
            shootingMode = ShootingMode.SHORT;
        }
        if (shootingMode == ShootingMode.SHORT) {
            rightLifterServo.setPosition(0);
            leftLifterServo.setPosition(0);
            launcherMotorSpeed = SLOW_SPEED;
        } else {
            rightLifterServo.setPosition(0.25);
            leftLifterServo.setPosition(0.25);
            launcherMotorSpeed = FAST_SPEED;
        }
    }

    public void setShortShootingMode() {
        shootingMode = ShootingMode.SHORT;
        rightLifterServo.setPosition(0);
        leftLifterServo.setPosition(0);
        launcherMotorSpeed = SLOW_SPEED;
    }


    public void stopFlywheel() {
        launcherMotor.setVelocity(0);
    }

    public void launch() {
        launcherServo.setPosition(1.0);
    }

    public void launchReset() {
        launcherServo.setPosition(0.0);
    }

    public void setPosition(double position) {
        leftLifterServo.setPosition(position);
        rightLifterServo.setPosition(position);
    }

    public void raiseLifter() {
        leftLifterServo.setPosition(1);
        rightLifterServo.setPosition(1);
    }

    public void lowerLifter() {
        leftLifterServo.setPosition(0);
        rightLifterServo.setPosition(0);
    }


    public double getLaunchMotorVelocity() {
        return launcherMotor.getVelocity();
    }

    public double getLauncherDesiredSpeed() {
        return launcherMotorSpeed;
    }
}
