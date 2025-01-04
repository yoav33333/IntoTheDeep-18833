package org.firstinspires.ftc.teamcode.pedroPathing.constants;

import PedroPathing.src.main.java.com.pedropathing.localization.constants.ThreeWheelConstants;
import PedroPathing.src.main.java.com.pedropathing.localization.Encoder;
public class LConstants {
    static {
        ThreeWheelConstants.forwardTicksToInches = .001989436789;
        ThreeWheelConstants.strafeTicksToInches = .001989436789;
        ThreeWheelConstants.turnTicksToInches = .001989436789;
        ThreeWheelConstants.leftY = 7;
        ThreeWheelConstants.rightY = -7;
        ThreeWheelConstants.strafeX = 0;
        ThreeWheelConstants.leftEncoder_HardwareMapName = "dfl";
        ThreeWheelConstants.rightEncoder_HardwareMapName = "drl";
        ThreeWheelConstants.strafeEncoder_HardwareMapName = "drr";
        ThreeWheelConstants.leftEncoderDirection = Encoder.REVERSE;
        ThreeWheelConstants.rightEncoderDirection = Encoder.REVERSE;
        ThreeWheelConstants.strafeEncoderDirection = Encoder.FORWARD;
    }
}




