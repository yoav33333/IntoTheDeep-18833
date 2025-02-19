package org.firstinspires.ftc.teamcode.subsystems

import com.pedropathing.follower.Follower
import com.qualcomm.robotcore.hardware.ColorRangeSensor
import com.qualcomm.robotcore.hardware.Servo
import dev.frozenmilk.dairy.cachinghardware.CachingServo
import dev.frozenmilk.dairy.core.FeatureRegistrar
import dev.frozenmilk.dairy.core.dependency.Dependency
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation
import dev.frozenmilk.dairy.core.util.OpModeLazyCell
import dev.frozenmilk.dairy.core.wrapper.Wrapper
import dev.frozenmilk.mercurial.Mercurial
import dev.frozenmilk.mercurial.commands.Lambda
import dev.frozenmilk.mercurial.subsystems.Subsystem

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import java.lang.annotation.Inherited


object clawSubsystem : Subsystem {
    override var dependency: Dependency<*> = Subsystem.DEFAULT_DEPENDENCY and
            SingleAnnotation(Mercurial.Attach::class.java)

    @Target(AnnotationTarget.CLASS)
    @Retention(AnnotationRetention.RUNTIME)
    @MustBeDocumented
    @Inherited
    annotation class Attach
    @JvmStatic
    val clawServo: Servo by OpModeLazyCell {
        val s =
            FeatureRegistrar.activeOpMode.hardwareMap.get(
                Servo::class.java, "claw servo"

        )
//        s.cachingTolerance = 0.001
        s
    }
    @JvmStatic
    var clawRotationServo: Servo by OpModeLazyCell {
        val s =
            FeatureRegistrar.activeOpMode.hardwareMap.get(
                Servo::class.java, "rotate servo"
        )
        s.setPosition(0.5)
        s
    }

    val colorDistSensor: ColorRangeSensor by OpModeLazyCell {
        FeatureRegistrar.activeOpMode.hardwareMap.get(
            ColorRangeSensor::class.java, "color claw"
        )
    }

    val closeingPose = 1.0
    val openingPose = 0.0

    val filter = 1
    var oldRead = 0.0
    var counter = 0
    fun readSensorDis(): Double {
        if (counter % filter == 0)
            oldRead = colorDistSensor.getDistance(DistanceUnit.MM)
        counter++
        return oldRead
    }

    fun closeClaw() {
        clawServo.setPosition(closeingPose)
    }

    var semiClose = 0.545
    fun closeClawP() {
        clawServo.setPosition(semiClose)
    }
//    val telemetry = MultipleTelemetry(FeatureRegistrar.activeOpMode.telemetry, FtcDashboard.getInstance().telemetry)
//
//
//    override fun postUserLoopHook(opMode: Wrapper) {
//        telemetry.addData("pose", semiClose)
//        telemetry.update()
//
//    }

    @JvmStatic
    fun openClaw() {
        clawServo.setPosition(openingPose)
    }

    val closeIfRead = Lambda("closeIfRead")
        .setFinish{ colorDistSensor.getDistance(DistanceUnit.MM)<35}
        .setEnd{closeClaw}
    val changeClawPos = Lambda("changeClawPos")
        .setRunStates(Wrapper.OpModeState.ACTIVE)
        .setEnd {
            if (clawServo.position == closeingPose) {
                openClaw()
            } else {
                closeClaw()
            }
        }
        .setFinish{ colorDistSensor.getDistance(DistanceUnit.MM)<38}
//    var check = true
//    val runCs = Lambda("rcs")
//        .setRunStates(Wrapper.OpModeState.ACTIVE)
//        .setInit{ check = true}
//
//    val stopCs = Lambda("scs")
//        .setRunStates(Wrapper.OpModeState.ACTIVE)
//        .setInit{ check = false
//            antonySubsystem.colorSensorData.cancel()}


    val rotateClawL = Lambda("rotate claw l")
        .setRunStates(Wrapper.OpModeState.ACTIVE)
        .setInit {
            clawRotationServo.setPosition(if (clawRotationServo.position <= 1.0) clawRotationServo.position + 0.25 else 1.0)
        }
    val rotateClawR = Lambda("rotate claw r")
        .setRunStates(Wrapper.OpModeState.ACTIVE)
        .setInit {
            clawRotationServo.setPosition(if (clawRotationServo.position >= 0.0) clawRotationServo.position - 0.25 else 0.0)
        }
    @JvmStatic
    val turnLeft = Lambda("turnLeft")
        .setRunStates(Wrapper.OpModeState.ACTIVE)
        .setInit { clawRotationServo.position = 1.0  }

    val turnRight = Lambda("turnRight")
        .setRunStates(Wrapper.OpModeState.ACTIVE)
        .setInit { clawRotationServo.position = 0.0 }
    val clawRotationRange = 180.0
    fun keepAngle(angle: Double, follower: Follower) = Lambda("keepAngle")
        .setExecute{
            clawRotationServo.position = (Math.toDegrees(follower.pose.heading) - angle +90)/ clawRotationRange
        }
    val resetAngleClaw = Lambda("resetAngleClaw")
        .setInit { clawRotationServo.setPosition(0.5) }
    @JvmStatic
    val openClaw = Lambda("openClaw")
        .setRunStates(Wrapper.OpModeState.ACTIVE)
        .setInit {
            openClaw()
        }
    @JvmStatic
    val closeClaw = Lambda("closeClaw")
        .setRunStates(Wrapper.OpModeState.ACTIVE)
        .setInit { closeClaw() }
    val closeClaw2 = Lambda("closeClaw2")
        .setRunStates(Wrapper.OpModeState.ACTIVE)
        .setInit { closeClawP() }

    override fun postUserInitHook(opMode: Wrapper) {
        openClaw()
        clawRotationServo.position = 0.5
    }
}