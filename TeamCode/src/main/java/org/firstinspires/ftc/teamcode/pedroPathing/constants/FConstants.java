package org.firstinspires.ftc.teamcode.pedroPathing.constants;


import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.localization.Localizers;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class FConstants {
    static {
        FollowerConstants.localizers = Localizers.THREE_WHEEL_IMU;

        FollowerConstants.leftFrontMotorName = "dfl";
        FollowerConstants.leftRearMotorName = "drl";
        FollowerConstants.rightFrontMotorName = "dfr";
        FollowerConstants.rightRearMotorName = "drr";

        FollowerConstants.leftFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
        FollowerConstants.leftRearMotorDirection = DcMotorSimple.Direction.FORWARD;
        FollowerConstants.rightFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.rightRearMotorDirection = DcMotorSimple.Direction.REVERSE;

        FollowerConstants.mass = 15;

        FollowerConstants.xMovement = 67.9649274103884;
        FollowerConstants.yMovement =  47.94003899767779;

        FollowerConstants.forwardZeroPowerAcceleration =  -41.815674186276155;
        FollowerConstants.lateralZeroPowerAcceleration = -71.31657454697638;

        FollowerConstants.translationalPIDFCoefficients.setCoefficients(0.36, 0, 0.045, 0);
        FollowerConstants.translationalPIDFFeedForward = 0.025;
        FollowerConstants.useSecondaryTranslationalPID = false;
        FollowerConstants.secondaryTranslationalPIDFCoefficients.setCoefficients(0.1, 0, 0.01, 0); // Not being used, @see useSecondaryTranslationalPID

        FollowerConstants.headingPIDFCoefficients.setCoefficients(1.94, 0, 0.14, 0);
        FollowerConstants.useSecondaryHeadingPID = false;
        FollowerConstants.secondaryHeadingPIDFCoefficients.setCoefficients(0.7, 0, 0.04, 0); // Not being used, @see useSecondaryHeadingPID

        FollowerConstants.drivePIDFCoefficients.setCoefficients(0.04, 0, 0.0115, 0.5, 0);
        FollowerConstants.useSecondaryDrivePID = false;
        FollowerConstants.secondaryDrivePIDFCoefficients.setCoefficients(0.1, 0, 0, 0.5, 0); // Not being used, @see useSecondaryDrivePID
        FollowerConstants.zeroPowerAccelerationMultiplier = 10;
        FollowerConstants.centripetalScaling = 0.0004;

        FollowerConstants.pathEndTimeoutConstraint = 100;
        FollowerConstants.pathEndTValueConstraint = 0.997;
        FollowerConstants.pathEndVelocityConstraint = 0.1;
        FollowerConstants.pathEndTranslationalConstraint = 0.1;
        FollowerConstants.pathEndHeadingConstraint = 0.005;
        FollowerConstants.useBrakeModeInTeleOp = true;
        FollowerConstants.useVoltageCompensationInAuto = true;
        FollowerConstants.useVoltageCompensationInTeleOp = false;
        FollowerConstants.nominalVoltage = 11;
        FollowerConstants.cacheInvalidateSeconds = 0.5;
    }
}
