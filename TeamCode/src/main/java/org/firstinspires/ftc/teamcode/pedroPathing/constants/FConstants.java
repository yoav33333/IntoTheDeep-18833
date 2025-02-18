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

        FollowerConstants.xMovement = 64.38234547128357;
        FollowerConstants.yMovement = 45.10405686807232;

        FollowerConstants.forwardZeroPowerAcceleration =  -55.71733464559506;
        FollowerConstants.lateralZeroPowerAcceleration = -99.3169995948494  ;

        FollowerConstants.translationalPIDFCoefficients.setCoefficients(0.215, 0, 0.02, 0);
        FollowerConstants.translationalPIDFFeedForward = 0.025;
        FollowerConstants.useSecondaryTranslationalPID = false;
        FollowerConstants.secondaryTranslationalPIDFCoefficients.setCoefficients(0.1, 0, 0.01, 0); // Not being used, @see useSecondaryTranslationalPID

        FollowerConstants.headingPIDFCoefficients.setCoefficients(1.95, 0, 0.14, 0);
        FollowerConstants.useSecondaryHeadingPID = false;
        FollowerConstants.secondaryHeadingPIDFCoefficients.setCoefficients(0.7, 0, 0.04, 0); // Not being used, @see useSecondaryHeadingPID

        FollowerConstants.drivePIDFCoefficients.setCoefficients(0.04, 0, 0.0025, 0.5, 0);
        FollowerConstants.useSecondaryDrivePID = false;
        FollowerConstants.secondaryDrivePIDFCoefficients.setCoefficients(0.1, 0, 0, 0.5, 0); // Not being used, @see useSecondaryDrivePID
        FollowerConstants.zeroPowerAccelerationMultiplier = 7;
        FollowerConstants.centripetalScaling = 0.0004;

        FollowerConstants.pathEndTimeoutConstraint = 500;
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
