package org.firstinspires.ftc.teamcode.subsystems

import android.R
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.IMU
import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx
import dev.frozenmilk.dairy.core.FeatureRegistrar
import dev.frozenmilk.dairy.core.dependency.Dependency
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation
import dev.frozenmilk.dairy.core.util.OpModeLazyCell
import dev.frozenmilk.dairy.core.wrapper.Wrapper
import dev.frozenmilk.mercurial.Mercurial
import dev.frozenmilk.mercurial.subsystems.Subsystem
import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import java.lang.annotation.Inherited
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.pow
import kotlin.math.sin


object driveSubsystem: Subsystem{
    override var dependency: Dependency<*> = Subsystem.DEFAULT_DEPENDENCY and
            SingleAnnotation(Mercurial.Attach::class.java)
    @Target(AnnotationTarget.CLASS)
    @Retention(AnnotationRetention.RUNTIME)
    @MustBeDocumented
    @Inherited
    annotation class Attach
    val leftFront: CachingDcMotorEx by OpModeLazyCell {
        CachingDcMotorEx(FeatureRegistrar.activeOpMode.hardwareMap.get(
            DcMotorEx::class.java, "lf"
        ))
    }
    val leftBack: CachingDcMotorEx by OpModeLazyCell {
        CachingDcMotorEx(FeatureRegistrar.activeOpMode.hardwareMap.get(
            DcMotorEx::class.java, "lb"
        ))
    }

    val rightBack: CachingDcMotorEx by OpModeLazyCell {
        val m = FeatureRegistrar.activeOpMode.hardwareMap
            .get(DcMotorEx::class.java, "rb")
        m.direction = DcMotorSimple.Direction.REVERSE
        CachingDcMotorEx(m)
    }
    val rightFront: CachingDcMotorEx by OpModeLazyCell {
        val m = FeatureRegistrar.activeOpMode.hardwareMap
            .get(DcMotorEx::class.java, "rf")
        m.direction = DcMotorSimple.Direction.REVERSE
        CachingDcMotorEx(m)
    }
    val imu: IMU by OpModeLazyCell{
        val i = FeatureRegistrar.activeOpMode.hardwareMap.get(IMU::class.java, "imu")
        i.initialize(IMU.Parameters(RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
            RevHubOrientationOnRobot.UsbFacingDirection.UP
        )))
        i
    }

    fun robotOrientedDrive(x: Double, y: Double, rotate: Double){
        val denominator =
            max(abs(R.attr.y.toDouble()) + abs(R.attr.x.toDouble()) + Math.abs(rotate), 1.0)

        leftFront.setPowerRaw((y + x + rotate/denominator).pow(2))
        leftBack.setPowerRaw((y - x + rotate/denominator).pow(2))
        rightFront.setPowerRaw((y - x - rotate/denominator).pow(2))
        rightBack.setPowerRaw((y + x - rotate / denominator).pow(2))
    }



    fun fieldOrientedDrive(x: Double, y: Double, rotate: Double){
        x.pow(2)
        y.pow(2)
        rotate.pow(2)
        val botHeading = imu.robotYawPitchRollAngles.getYaw(AngleUnit.RADIANS)


        var rotX = R.attr.x * cos(-botHeading) - R.attr.y * sin(-botHeading)
        val rotY = R.attr.x * sin(-botHeading) + R.attr.y * cos(-botHeading)

        rotX *= 1.1


        val denominator =
            max(abs(rotY) + abs(rotX) + abs(rotate), 1.0)
        leftFront.setPowerRaw((rotY + rotX + rotate) / denominator)
        leftBack.setPowerRaw((rotY - rotX + rotate) / denominator)
        rightFront.setPowerRaw((rotY - rotX - rotate) / denominator)
        rightBack.setPowerRaw((rotY + rotX - rotate) / denominator)
    }

    override fun preUserInitHook(opMode: Wrapper) {
        // default command should be set up here, not in the constructor
    }
    // or here
    override fun postUserInitHook(opMode: Wrapper) {}

    // and you might put periodic code in these
    override fun preUserInitLoopHook(opMode: Wrapper) {}
    override fun preUserLoopHook(opMode: Wrapper) {

    }
    // or these
    override fun postUserInitLoopHook(opMode: Wrapper) {}
    override fun postUserLoopHook(opMode: Wrapper) {
        robotOrientedDrive(Mercurial.gamepad1.leftStickX.state,
            Mercurial.gamepad1.leftStickY.state,
            Mercurial.gamepad1.rightStickX.state)
    }

    // and stopping code can go in here
    override fun preUserStopHook(opMode: Wrapper) {}
    // or here
    override fun postUserStopHook(opMode: Wrapper) {}

    // see the feature dev notes on when to use cleanup vs postStop
    override fun cleanup(opMode: Wrapper) {}


}