package org.firstinspires.ftc.teamcode.subsystems.drive

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import dev.frozenmilk.dairy.core.dependency.Dependency
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation
import dev.frozenmilk.mercurial.Mercurial
import dev.frozenmilk.mercurial.commands.Command
import dev.frozenmilk.mercurial.subsystems.SDKSubsystem
import dev.frozenmilk.mercurial.subsystems.Subsystem
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveCommands.driveCommand
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveVariables.headingSupplier
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveVariables.imuAngleOffset
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveVariables.maxPower
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveVariables.robotCentricSupplier
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveVariables.xSupplier
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveVariables.ySupplier
import org.firstinspires.ftc.teamcode.subsystems.followerSubsystem.gamepad1
import org.firstinspires.ftc.teamcode.subsystems.followerSubsystem.gamepad2
import org.firstinspires.ftc.teamcode.subsystems.followerSubsystem.headingPow
import org.firstinspires.ftc.teamcode.util.SuperIMU
import org.firstinspires.ftc.teamcode.util.SuperMotor
import java.lang.annotation.Inherited
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sin


object DriveHardware:SDKSubsystem() {
    override var dependency: Dependency<*> = Subsystem.DEFAULT_DEPENDENCY and
            SingleAnnotation(Mercurial.Attach::class.java)

    @Target(AnnotationTarget.CLASS)
    @Retention(AnnotationRetention.RUNTIME)
    @MustBeDocumented
    @Inherited
    annotation class Attach

    val leftFront = SuperMotor("dfl")
        .setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
        .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
        .get()
    val leftBack = SuperMotor("drl")
        .setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
        .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
        .get()
    val rightFront = SuperMotor("dfr")
        .setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
        .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
        .setDirection(DcMotorSimple.Direction.REVERSE)
        .get()
    val rightBack = SuperMotor("drr")
        .setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
        .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
        .setDirection(DcMotorSimple.Direction.REVERSE)
        .get()
    val imu = SuperIMU("imu")
        .setOrientationOnRobot(RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD))
        .get()

    val getPureIMUHeading = { imu.robotYawPitchRollAngles.yaw }
    val getIMUHeading = { imu.robotYawPitchRollAngles.yaw + imuAngleOffset }
    fun setIMUHeading(heading: Double) {
        imuAngleOffset = heading - getIMUHeading()
    }

    fun setPower(leftFront: Double, leftBack: Double, rightFront: Double, rightBack: Double) {
        this.leftFront.power = leftFront
        this.leftBack.power = leftBack
        this.rightFront.power = rightFront
        this.rightBack.power = rightBack
    }

    fun drive(x: Double, y: Double, rotation: Double, robotCentric: Boolean = false) {
        var heading = Math.toRadians(getIMUHeading())
        var rotX = x
        var rotY = y
        if (!robotCentric){
            rotX = x * cos(-heading) - y * sin(-heading)
            rotY = x * sin(-heading) + y * cos(-heading)
        }
        var denominator = Math.max(abs(rotY) + abs(rotX) + Math.abs(rotation), 1.0)
        var frontLeftPower: Double = (rotY + rotX + rotation) / denominator * maxPower
        var backLeftPower: Double = (rotY - rotX + rotation) / denominator * maxPower
        var frontRightPower: Double = (rotY - rotX - rotation) / denominator * maxPower
        var backRightPower: Double = (rotY + rotX - rotation) / denominator * maxPower

        setPower(frontLeftPower, backLeftPower, frontRightPower, backRightPower)
    }

    override var defaultCommand: Command? = driveCommand(xSupplier, ySupplier, headingSupplier, robotCentricSupplier)







}