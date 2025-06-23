package org.firstinspires.ftc.teamcode.subsystems.drive

import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.IMU
import dev.frozenmilk.dairy.core.dependency.Dependency
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation
import dev.frozenmilk.dairy.core.util.OpModeLazyCell
import dev.frozenmilk.dairy.core.wrapper.Wrapper
import dev.frozenmilk.dairy.pasteurized.Pasteurized.gamepad1
import dev.frozenmilk.mercurial.Mercurial
import dev.frozenmilk.mercurial.subsystems.SDKSubsystem
import dev.frozenmilk.mercurial.subsystems.Subsystem
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveCommands.driveCommand
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveVariables.headingSupplier
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveVariables.imuAngleOffset
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveVariables.robotCentricSupplier
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveVariables.xSupplier
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveVariables.ySupplier
import org.firstinspires.ftc.teamcode.util.SuperIMU
import org.firstinspires.ftc.teamcode.util.SuperMotor
import java.lang.annotation.Inherited
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.sin


object DriveHardware:SDKSubsystem() {
    override var dependency: Dependency<*> = Subsystem.DEFAULT_DEPENDENCY and
            SingleAnnotation(Mercurial.Attach::class.java)

    @Target(AnnotationTarget.CLASS)
    @Retention(AnnotationRetention.RUNTIME)
    @MustBeDocumented
    @Inherited
    annotation class Attach

    val leftFront by OpModeLazyCell{SuperMotor("drive 1")
        .setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
        .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
        .get()}
    val leftBack by OpModeLazyCell{SuperMotor("drive 2")
        .setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
        .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
        .get()}
    val rightFront by OpModeLazyCell{SuperMotor("drive 4")
        .setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
        .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
        .setDirection(DcMotorSimple.Direction.REVERSE)
        .get()}
    val rightBack by OpModeLazyCell{SuperMotor("drive 3")
        .setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
        .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
        .setDirection(DcMotorSimple.Direction.REVERSE)
        .get()}
    val imu by OpModeLazyCell{ SuperIMU("imu")
        .setOrientationOnRobot(RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD))
        .get()}

    fun getPureIMUHeading() =  imu.robotYawPitchRollAngles.yaw
    fun getIMUHeading() = imu.robotYawPitchRollAngles.yaw + imuAngleOffset

    fun setIMUHeading(heading: Double) {
        imuAngleOffset = heading - getPureIMUHeading()
    }
    fun setOrientationOnRobot(orientationOnRobot: RevHubOrientationOnRobot) {
        imu.initialize(IMU.Parameters(orientationOnRobot))
    }
    fun setBreak() {
        leftFront.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        leftBack.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        rightFront.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        rightBack.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    }


    fun setRunMode(){
        leftFront.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        leftBack.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        rightFront.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        rightBack.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }

    fun setPower(leftFront: Double, leftBack: Double, rightFront: Double, rightBack: Double) {
        this.leftFront.power = leftFront
        this.leftBack.power = leftBack
        this.rightFront.power = -rightFront
        this.rightBack.power = -rightBack
    }

    fun drive(robotCentric: Boolean = false) {
        val y = -ySupplier.asDouble // Remember, Y stick value is reversed
        val x = xSupplier.asDouble
        val rx = headingSupplier.asDouble

        val botHeading = -Math.toRadians(getIMUHeading())
        var rotX = 0.0
        var rotY = 0.0
//        if (robotCentric) {
            // Rotate the movement direction counter to the bot's rotation
            rotX = x * cos(-botHeading) - y * sin(-botHeading)
            rotY = x * sin(-botHeading) + y * cos(-botHeading)
//        }
        rotX = rotX * 1.1 // Counteract imperfect strafing


        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        val denominator = max(abs(rotY) + abs(rotX) + abs(rx), 1.0)
        val frontLeftPower = (rotY + rotX + rx) / denominator
        val backLeftPower = (rotY - rotX + rx) / denominator
        val frontRightPower = (rotY - rotX - rx) / denominator
        val backRightPower = (rotY + rotX - rx) / denominator
        setPower(frontLeftPower, backLeftPower, frontRightPower, backRightPower)

    }

//    override var defaultCommand: Command? = driveCommand(xSupplier, ySupplier, headingSupplier, robotCentricSupplier)
    override fun postUserStartHook(opMode: Wrapper) {
        setBreak()
        setRunMode()
        setOrientationOnRobot(RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD))
        driveCommand(xSupplier, ySupplier, headingSupplier, robotCentricSupplier).schedule()
    }






}