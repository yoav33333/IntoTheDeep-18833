package org.firstinspires.ftc.teamcode.pedroPathing.constants;

import static kotlin.math.MathKt.PI;

import com.pedropathing.localization.Encoder;
import com.pedropathing.localization.constants.ThreeWheelIMUConstants;
import com.pedropathing.localization.constants.ThreeWheelIMUConstants;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;


public class LConstants {
    static {
        ThreeWheelIMUConstants.IMU_HardwareMapName = "imu";
        ThreeWheelIMUConstants.IMU_Orientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
        ThreeWheelIMUConstants.forwardTicksToInches = 2 * PI / 8192;
        ThreeWheelIMUConstants.strafeTicksToInches = 2 * PI / 8192;
        ThreeWheelIMUConstants.turnTicksToInches = 2 * PI / 8192;
        ThreeWheelIMUConstants.leftY =7.2;
//        ThreeWheelIMUConstants.rightY = -6.77;
        //7.3, 6.75
        ThreeWheelIMUConstants.rightY = -6.73;
//        21.7, 43.2
//        ThreeWheelIMUConstants.strafeX = -0.04;
        ThreeWheelIMUConstants.strafeX = 0.618;
        ThreeWheelIMUConstants.leftEncoder_HardwareMapName = "dfl";
        ThreeWheelIMUConstants.rightEncoder_HardwareMapName = "drl";
        ThreeWheelIMUConstants.strafeEncoder_HardwareMapName = "drr";
        ThreeWheelIMUConstants.leftEncoderDirection = Encoder.FORWARD;
        ThreeWheelIMUConstants.rightEncoderDirection = Encoder.FORWARD;
        ThreeWheelIMUConstants.strafeEncoderDirection = Encoder.REVERSE;
    }
}




