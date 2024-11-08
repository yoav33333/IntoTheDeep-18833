package org.firstinspires.ftc.teamcode.subsystems

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

    val leftClawServo: CachingServo by OpModeLazyCell {
        val s = CachingServo(
            FeatureRegistrar.activeOpMode.hardwareMap.get(
                Servo::class.java, "LeftServo"
            )
        )
        s
    }
    val rightClawServo: CachingServo by OpModeLazyCell {
        val s = CachingServo(
            FeatureRegistrar.activeOpMode.hardwareMap.get(
                Servo::class.java, "RightServo"
            )
        )
        s
    }
    val clawRotationServo: CachingServo by OpModeLazyCell {
        val s = CachingServo(
            FeatureRegistrar.activeOpMode.hardwareMap.get(
                Servo::class.java, "Axonion"
            )
        )
        s.setPositionRaw(0.5)
        s
    }
    val closeingPose = 1.0
    val openingPose = 0.0

    fun closeClaw() {
        leftClawServo.setPosition(openingPose)
        rightClawServo.setPosition(closeingPose)
    }

    fun openClaw() {
        leftClawServo.setPosition(closeingPose)
        rightClawServo.setPositionRaw(openingPose)
    }


    var changeClawPos = Lambda("changeClawPos")
        .setRunStates(Wrapper.OpModeState.ACTIVE)
        .addRequirements(clawSubsystem)
        .setInit{
            if (leftClawServo.position == openingPose) {
                openClaw()
            } else {
                closeClaw()
            }
        }


    val rotateClaw = Lambda("rotate claw")
        .setRunStates(Wrapper.OpModeState.ACTIVE)
//        .addRequirements(clawSubsystem)
        .setExecute{ clawRotationServo.position  += Mercurial.gamepad2.leftStickX.state*0.007 }

    val resetClaw = Lambda("resetClaw")
//        .addRequirements(clawSubsystem)
        .setRunStates(Wrapper.OpModeState.ACTIVE)
        .setInit{clawRotationServo.position = 0.5}

    val openClaw = Lambda("openClaw")
//        .addRequirements(clawSubsystem)
        .setRunStates(Wrapper.OpModeState.ACTIVE)
        .setInit{ clawSubsystem.openClaw()}
    val closeClaw = Lambda("closeClaw")
//        .addRequirements(clawSubsystem)
        .setRunStates(Wrapper.OpModeState.ACTIVE)
        .setInit{ clawSubsystem.closeClaw()}

    override fun postUserInitHook(opMode: Wrapper) {
        openClaw()
        resetClaw
    }
}