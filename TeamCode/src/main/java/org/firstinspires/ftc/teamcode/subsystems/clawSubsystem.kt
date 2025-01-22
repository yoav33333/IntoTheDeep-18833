package org.firstinspires.ftc.teamcode.subsystems

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

    val clawServo: CachingServo by OpModeLazyCell {
        val s = CachingServo(
            FeatureRegistrar.activeOpMode.hardwareMap.get(
                Servo::class.java, "claw servo"
            )
        )
        s.cachingTolerance = 0.001
        s
    }

    val clawRotationServo: CachingServo by OpModeLazyCell {
        val s = CachingServo(
            FeatureRegistrar.activeOpMode.hardwareMap.get(
                Servo::class.java, "rotate servo"
            )
        )
        s.cachingTolerance = 0.001
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

    fun openClaw() {
        clawServo.setPosition(openingPose)
    }


    val changeClawPos = Lambda("changeClawPos")
        .setRunStates(Wrapper.OpModeState.ACTIVE)
        .setInit {
            if (clawServo.position == closeingPose) {
                openClaw()
            } else {
                closeClaw()
            }
        }
//    var check = true
//    val runCs = Lambda("rcs")
//        .setRunStates(Wrapper.OpModeState.ACTIVE)
//        .setInit{ check = true}
//
//    val stopCs = Lambda("scs")
//        .setRunStates(Wrapper.OpModeState.ACTIVE)
//        .setInit{ check = false
//            antonySubsystem.colorSensorData.cancel()}


    val rotateClawR = Lambda("rotate claw r")
        .setRunStates(Wrapper.OpModeState.ACTIVE)
        .setInit {
            clawRotationServo.setPosition(if (clawRotationServo.position < 0.9) clawRotationServo.position + 0.2 else 0.9)
        }
    val rotateClawL = Lambda("rotate claw l")
        .setRunStates(Wrapper.OpModeState.ACTIVE)
        .setInit {
            clawRotationServo.setPosition(if (clawRotationServo.position > 0.1) clawRotationServo.position - 0.2 else 0.1)
        }
    val turnLeft = Lambda("turnLeft")
        .setRunStates(Wrapper.OpModeState.ACTIVE)
        .setInit { clawRotationServo.setPosition(0.1) }

    val turnRight = Lambda("turnRight")
        .setRunStates(Wrapper.OpModeState.ACTIVE)
        .setInit { clawRotationServo.setPosition(0.9) }


    val resetAngleClaw = Lambda("resetAngleClaw")
        .setInit { clawRotationServo.setPosition(0.5) }

    val openClaw = Lambda("openClaw")
        .setRunStates(Wrapper.OpModeState.ACTIVE)
        .setInit {
            openClaw()
        }

    val closeClaw = Lambda("closeClaw")
        .setRunStates(Wrapper.OpModeState.ACTIVE)
        .setInit { closeClaw() }
    val closeClaw2 = Lambda("closeClaw2")
        .setRunStates(Wrapper.OpModeState.ACTIVE)
        .setInit { closeClawP() }

    override fun postUserInitHook(opMode: Wrapper) {
        openClaw()
        clawRotationServo.setPosition(0.5)
    }
}