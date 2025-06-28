package org.firstinspires.ftc.teamcode.pedroPathing.constants;


import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.localization.Localizers;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class FConstants {
    static {
        FollowerConstants.localizers = Localizers.THREE_WHEEL_IMU;

        FollowerConstants.leftFrontMotorName = "drive 2";
        FollowerConstants.leftRearMotorName = "drive 1";
        FollowerConstants.rightFrontMotorName = "drive 3";
        FollowerConstants.rightRearMotorName = "drive 4";

        FollowerConstants.leftFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
        FollowerConstants.leftRearMotorDirection = DcMotorSimple.Direction.FORWARD;
        FollowerConstants.rightFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.rightRearMotorDirection = DcMotorSimple.Direction.REVERSE;

        FollowerConstants.mass = 16;

        FollowerConstants.xMovement = 68.21018109820824;
        FollowerConstants.yMovement = 52.539635081616076;

        FollowerConstants.forwardZeroPowerAcceleration = -41.04315605123944;
        FollowerConstants.lateralZeroPowerAcceleration = -82.82592063867189;

        FollowerConstants.translationalPIDFCoefficients.setCoefficients(0.18, 0, 0.03, 0);
        FollowerConstants.translationalPIDFFeedForward = 0.025;
        FollowerConstants.useSecondaryTranslationalPID = false;
        FollowerConstants.secondaryTranslationalPIDFCoefficients.setCoefficients(0.1, 0, 0.01, 0); // Not being used, @see useSecondaryTranslationalPID

        FollowerConstants.headingPIDFCoefficients.setCoefficients(1.2, 0, 0.11, 0);
        FollowerConstants.useSecondaryHeadingPID = false;
        FollowerConstants.secondaryHeadingPIDFCoefficients.setCoefficients(0.7, 0, 0.04, 0); // Not being used, @see useSecondaryHeadingPID

        FollowerConstants.drivePIDFCoefficients.setCoefficients(0.029, 0, 0.030, 0.9, 0);
        FollowerConstants.useSecondaryDrivePID = false;
        FollowerConstants.secondaryDrivePIDFCoefficients.setCoefficients(0.1, 0, 0, 0.5, 0); // Not being used, @see useSecondaryDrivePID
        FollowerConstants.zeroPowerAccelerationMultiplier = 7;
        FollowerConstants.centripetalScaling = 0.0004;
        //
        FollowerConstants.automaticHoldEnd = true;
        //
        FollowerConstants.turnHeadingErrorThreshold = 0.01;
        FollowerConstants.pathEndTimeoutConstraint = 10;
        FollowerConstants.pathEndTValueConstraint = 0.996;
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
