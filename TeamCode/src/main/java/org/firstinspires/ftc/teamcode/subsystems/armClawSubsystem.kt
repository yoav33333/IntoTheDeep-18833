package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.robotcore.hardware.Servo
import dev.frozenmilk.dairy.cachinghardware.CachingServo
import dev.frozenmilk.dairy.core.FeatureRegistrar
import dev.frozenmilk.dairy.core.dependency.Dependency
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation
import dev.frozenmilk.dairy.core.util.OpModeLazyCell
import dev.frozenmilk.mercurial.Mercurial
import dev.frozenmilk.mercurial.commands.Lambda
import dev.frozenmilk.mercurial.commands.groups.Sequential
import dev.frozenmilk.mercurial.commands.util.Wait
import dev.frozenmilk.mercurial.subsystems.Subsystem
import java.lang.annotation.Inherited


object armClawSubsystem : Subsystem {
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
    val armOut = 0.02
    val armIn = 0.95
    val transfareState = 0.06
    val postTransfareState = 0.6
    val intakeState = 1.0


    val angleTransfer = Lambda("angleTransfer")
        .setInit {
            angleClawServo.setPosition(transfareState)
        }
    val anglePostTransfer = Lambda("anglePostTransfer")
        .setInit {
            angleClawServo.setPosition(postTransfareState)
        }
    val angleIntake = Lambda("angleIntake")
        .setInit {
            angleClawServo.setPosition(intakeState)
        }
    val moveArmOut = Lambda("moveArmOut")
        .setInit {
            armClawServo.setPosition(armOut)
        }
    val moveArmIn = Lambda("moveArmIn")
        .setInit {
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