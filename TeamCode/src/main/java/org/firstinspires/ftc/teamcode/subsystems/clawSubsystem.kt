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
import java.lang.annotation.Inherited


object clawSubsystem: Subsystem {
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

    val colorDistSensor: ColorRangeSensor by OpModeLazyCell{
        FeatureRegistrar.activeOpMode.hardwareMap.get(
            ColorRangeSensor::class.java, "color claw"
        )
    }

    val closeingPose = 1.0
    val openingPose = 0.0

    fun closeClaw() {
        clawServo.setPosition(closeingPose)
    }

    fun openClaw() {
        clawServo.setPosition(openingPose)
    }


    var changeClawPos = Lambda("changeClawPos")
        .setRunStates(Wrapper.OpModeState.ACTIVE)
        .setInit{
            if (clawServo.position == closeingPose) {
                openClaw()
            } else {
                closeClaw()
            }
        }


    val rotateClaw = Lambda("rotate claw")
        .setRunStates(Wrapper.OpModeState.ACTIVE)
        .setExecute{ clawRotationServo.position  += Mercurial.gamepad2.leftStickX.state*0.1
        }
    val turnLeft = Lambda("turnLeft")
        .setRunStates(Wrapper.OpModeState.ACTIVE)
        .setInit{clawRotationServo.position = 0.1}

    val turnRight = Lambda("turnRight")
        .setRunStates(Wrapper.OpModeState.ACTIVE)
        .setInit{clawRotationServo.position = 0.9}



    val resetAngleClaw = Lambda("resetAngleClaw")
        .setRunStates(Wrapper.OpModeState.ACTIVE)
        .setInit{clawRotationServo.position = 0.5}

    val openClaw = Lambda("openClaw")
        .setRunStates(Wrapper.OpModeState.ACTIVE)
        .setInit{
            openClaw()}
    val closeClaw = Lambda("closeClaw")
        .setRunStates(Wrapper.OpModeState.ACTIVE)
        .setInit{ closeClaw()}

    override fun postUserInitHook(opMode: Wrapper) {
        openClaw()
        clawRotationServo.position = 0.5
    }
}