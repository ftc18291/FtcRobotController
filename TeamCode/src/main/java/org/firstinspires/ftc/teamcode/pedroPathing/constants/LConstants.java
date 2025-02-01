package pedroPathing.constants;

import com.pedropathing.localization.*;
import com.pedropathing.localization.constants.*;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public class LConstants {
    final static double ODOMETRY_WHEEL_DIAMETER_MM = 48.0;
    final static double ODOMETRY_WHEEL_CIRCUMFERENCE_MM = ODOMETRY_WHEEL_DIAMETER_MM * Math.PI;
    final static double ENCODER_COUNTS_PER_REVOLUTION = 2000;
    final static double MM_IN_AN_INCH = 25.4;
    final static double DISTANCE_PER_ENCODER_TICK_INCHES = ODOMETRY_WHEEL_CIRCUMFERENCE_MM / MM_IN_AN_INCH / ENCODER_COUNTS_PER_REVOLUTION;

    static {
        ThreeWheelIMUConstants.forwardTicksToInches = 0.002968434;
        ThreeWheelIMUConstants.strafeTicksToInches = 0.002968434;
        ThreeWheelIMUConstants.turnTicksToInches = 0.00203;
        ThreeWheelIMUConstants.leftY = 3.5;
        ThreeWheelIMUConstants.rightY = -3.5;
        ThreeWheelIMUConstants.strafeX = -0.5;
        ThreeWheelIMUConstants.leftEncoder_HardwareMapName = "leftRear";
        ThreeWheelIMUConstants.rightEncoder_HardwareMapName = "rightRear";
        ThreeWheelIMUConstants.strafeEncoder_HardwareMapName = "leftFront";
        ThreeWheelIMUConstants.leftEncoderDirection = Encoder.REVERSE;
        ThreeWheelIMUConstants.rightEncoderDirection = Encoder.FORWARD;
        ThreeWheelIMUConstants.strafeEncoderDirection = Encoder.FORWARD;
        ThreeWheelIMUConstants.IMU_HardwareMapName = "imu";
        ThreeWheelIMUConstants.IMU_Orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD);

    }
}




