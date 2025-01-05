package org.firstinspires.ftc.teamcode.mechwarriors.behaviors;

import static java.lang.Thread.sleep;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechwarriors.hardware.Robot;

public class DriveHeadingPID extends Behavior {
    Robot robot;
    int heading;
    double rawDistance;
    int distance;

    double minStartSpeed = 0.1;
    int maxAccelerationDistance = 600;
    int decelerationDistance = 1000;

    ElapsedTime pidTimer = new ElapsedTime();
    ElapsedTime speedTimer = new ElapsedTime();
    double lastSpeedPosition;
    double lastSpeedTime;

    double speed;
    double lastSpeed;

    double Kp = 0.9;
    double Ki = 0.001;
    double Kd = 0.002;

    double lastError = 0;
    double integral = 0;

    public DriveHeadingPID(Telemetry telemetry, Robot robot, int heading, double distance, double speed) {
        this.robot = robot;
        this.telemetry = telemetry;
        this.name = "Drive Heading = [heading: " + heading + "°] [distance: " + distance + "] [speed: " + speed + "]";
        this.heading = heading;
        this.rawDistance = distance;
        this.distance = robot.calculateDriveTicks(distance);
        this.speed = speed;
        this.lastSpeed = 0;
    }

    @Override
    public void start() {
        robot.resetMotorTicks();
        robot.getSparkFunOTOS().resetTracking();
        pidTimer.reset();
        speedTimer.reset();
        lastSpeedPosition = robot.getDriveTicks();
        lastSpeedTime = speedTimer.seconds();
        run();
    }

    @Override
    public void run() {

        double currentPosition = robot.getDriveTicks();
        double remainingDistance = distance - currentPosition;

        if (remainingDistance <= 0) {
            robot.stop();
            isDone = true;
        } else {
            telemetry.addData("robot ticks", currentPosition);
            telemetry.addData("distance", distance);
           // SparkFunOTOS.Pose2D otosPos = robot.getSparkFunOTOS().getPosition();
           // telemetry.addData("otosPos", "x: " + otosPos.x + ", y: " + otosPos.y);

            //if (otosPos.x <= rawDistance) {
            if (currentPosition <= distance) {

                double targetSpeed;
                if (currentPosition < maxAccelerationDistance) {
                    targetSpeed = Math.max((currentPosition / maxAccelerationDistance) * speed, minStartSpeed);

                } else if (remainingDistance < decelerationDistance) {
                    targetSpeed = (remainingDistance / decelerationDistance) * speed;
                } else {
                    targetSpeed = speed;
                }

                double currentSpeed = getCurrentSpeed();
                double error = targetSpeed - currentSpeed;
                double deltaTime = pidTimer.seconds();
                integral += error * deltaTime;
                double derivative = (error - lastError) / deltaTime;
                double power = Kp * error + Ki * integral + Kd * derivative;

                double robotHeading = robot.getHeading();
                telemetry.addData("robotHeading", robotHeading);
                double steeringCorrection = (robotHeading - heading) * 0.02;
                telemetry.addData("steeringCorrection", steeringCorrection);
                //double pidSpeed = pidControl(this.distance, ticks) ;
                //telemetry.addData("PID speed", pidSpeed);
                telemetry.addData("target power", power);
                robot.drive(0, power, steeringCorrection);

                lastError = error;
                pidTimer.reset();
            }
        }
    }

    private double getCurrentSpeed()  {
        double currentPosition = robot.getDriveTicks();
        double currentTime = speedTimer.seconds();
        double speed = (currentPosition = lastSpeedPosition) / (currentTime - lastSpeedTime);
        lastSpeedPosition = currentPosition;
        lastSpeedTime = currentTime;
        return speed;
    }

    public double pidControl(double reference, double state) {
//        telemetry.addData("reference", reference);
//        telemetry.addData("state", state);
//        double error = reference - state;
//        telemetry.addData("error", error);
//        integral += error * timer.seconds();
//        telemetry.addData("integralSum", integral);
//        double derivative = (error / lastError) / timer.seconds();
//        telemetry.addData("derivative", derivative);
//        lastError = error;
//
//        timer.reset();
//
//        double output = (error * Kp) + (derivative * Kd) + (integral * Ki);
//        telemetry.addData("output", output);
//        return output;
        return 0.0;
    }
}
