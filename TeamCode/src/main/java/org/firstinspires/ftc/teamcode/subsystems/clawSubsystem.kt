package org.firstinspires.ftc.teamcode.subsystems

import android.R
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.IMU
import com.qualcomm.robotcore.hardware.Servo
import dev.frozenmilk.dairy.cachinghardware.CachingCRServo
import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx
import dev.frozenmilk.dairy.cachinghardware.CachingServo
import dev.frozenmilk.dairy.core.FeatureRegistrar
import dev.frozenmilk.dairy.core.dependency.Dependency
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation
import dev.frozenmilk.dairy.core.util.OpModeLazyCell
import dev.frozenmilk.dairy.core.wrapper.Wrapper
import dev.frozenmilk.dairy.pasteurized.Pasteurized
import dev.frozenmilk.mercurial.Mercurial
import dev.frozenmilk.mercurial.commands.Lambda
import dev.frozenmilk.mercurial.subsystems.Subsystem
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.subsystems.clawSubsystem.clawRotationServo
import java.lang.annotation.Inherited
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.pow
import kotlin.math.sin


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
        s.setPositionRaw(openingPose)
        s
    }
    val rightClawServo: CachingServo by OpModeLazyCell {
        val s = CachingServo(
            FeatureRegistrar.activeOpMode.hardwareMap.get(
                Servo::class.java, "RightServo"
            )
        )
        s.setPositionRaw(openingPose)
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
        leftClawServo.setPosition(closeingPose)
        rightClawServo.setPositionRaw(openingPose)
    }

    fun openClaw() {
        leftClawServo.setPosition(openingPose)
        rightClawServo.setPosition(closeingPose)
    }


    var changeClawPos = Lambda("changeClawPos")
        .setRunStates(Wrapper.OpModeState.ACTIVE)
        .addRequirements(clawSubsystem)
        .setInit{
            if (leftClawServo.position == closeingPose) {
                openClaw()
            } else {
                closeClaw()
            }
        }


    var rotateClaw = Lambda("rotate claw")
        .setRunStates(Wrapper.OpModeState.ACTIVE)
        .addRequirements(clawSubsystem)
        .setExecute{ clawRotationServo.position  += Mercurial.gamepad2.leftStickX.state*0.05 }
}