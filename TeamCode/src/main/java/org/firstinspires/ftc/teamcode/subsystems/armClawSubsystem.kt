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
import dev.frozenmilk.mercurial.commands.groups.Sequential
import dev.frozenmilk.mercurial.commands.util.Wait
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


object armClawSubsystem: Subsystem {
    override var dependency: Dependency<*> = Subsystem.DEFAULT_DEPENDENCY and
            SingleAnnotation(Mercurial.Attach::class.java)

    @Target(AnnotationTarget.CLASS)
    @Retention(AnnotationRetention.RUNTIME)
    @MustBeDocumented
    @Inherited
    annotation class Attach

    val armClawServo: CachingServo by OpModeLazyCell {
        val s = CachingServo(
            FeatureRegistrar.activeOpMode.hardwareMap.get(
                Servo::class.java, "4 bar s"
            )
        )
        s
    }
    val angleClawServo: CachingServo by OpModeLazyCell {
        val s = CachingServo(
            FeatureRegistrar.activeOpMode.hardwareMap.get(
                Servo::class.java, "arm s"
            )
        )
        s.direction = Servo.Direction.REVERSE
        s
    }
    val armOut = 0.09
    val armIn = 1.0
    val transfareState = 0.0
    val intakeState = 1.0


    val angleTransfer = Lambda("angleTransfer")
        .setInit{
            angleClawServo.setPosition(transfareState)
        }
    val angleIntake = Lambda("angleIntake")
        .setInit{
            angleClawServo.setPosition(intakeState)
        }
    val moveArmOut = Lambda("moveArmOut")
        .setInit{
            armClawServo.setPosition(armOut)
        }
    val moveArmIn = Lambda("moveArmIn")
        .setInit{
            armClawServo.setPosition(armIn)
        }

    val openClawArm = Sequential(
        moveArmOut,
        Wait(0.2),
        angleIntake
    )
    val closeClawArm = Sequential(
        angleTransfer,
        Wait(0.2),
        moveArmIn
    )


}