package org.firstinspires.ftc.teamcode.pedroPathing.constants;

import static kotlin.math.MathKt.PI;

import com.pedropathing.localization.Encoder;
import com.pedropathing.localization.constants.ThreeWheelConstants;


public class LConstants {
    static {
        ThreeWheelConstants.forwardTicksToInches = 2*PI/8192;
        ThreeWheelConstants.strafeTicksToInches = 2*PI/8192;
        ThreeWheelConstants.turnTicksToInches = 2*PI/8192;
        ThreeWheelConstants.leftY = 7;
        ThreeWheelConstants.rightY = -7;
        ThreeWheelConstants.strafeX = 0;
        ThreeWheelConstants.leftEncoder_HardwareMapName = "dfl";
        ThreeWheelConstants.rightEncoder_HardwareMapName = "drl";
        ThreeWheelConstants.strafeEncoder_HardwareMapName = "drr";
        ThreeWheelConstants.leftEncoderDirection = Encoder.FORWARD;
        ThreeWheelConstants.rightEncoderDirection = Encoder.FORWARD;
        ThreeWheelConstants.strafeEncoderDirection = Encoder.REVERSE;
    }
}




