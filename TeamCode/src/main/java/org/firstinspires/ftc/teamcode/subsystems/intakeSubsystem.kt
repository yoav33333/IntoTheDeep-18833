package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.Servo
import dev.frozenmilk.dairy.core.FeatureRegistrar
import dev.frozenmilk.dairy.core.dependency.Dependency
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation
import dev.frozenmilk.dairy.core.util.OpModeLazyCell
import dev.frozenmilk.mercurial.Mercurial
import dev.frozenmilk.mercurial.commands.Lambda
import dev.frozenmilk.mercurial.subsystems.SDKSubsystem
import dev.frozenmilk.mercurial.subsystems.Subsystem
import java.lang.annotation.Inherited

@Config
object intakeSubsystem : SDKSubsystem() {
    override var dependency: Dependency<*> = Subsystem.DEFAULT_DEPENDENCY and
            SingleAnnotation(Mercurial.Attach::class.java)

    @Target(AnnotationTarget.CLASS)
    @Retention(AnnotationRetention.RUNTIME)
    @MustBeDocumented
    @Inherited
    annotation class Attach

    val leftWheel: Servo by OpModeLazyCell {
        val s =
            FeatureRegistrar.activeOpMode.hardwareMap.get(
                Servo::class.java, "leftWheel"

            )
        s
    }
    val rightWheel: Servo by OpModeLazyCell {
        val s =
            FeatureRegistrar.activeOpMode.hardwareMap.get(
                Servo::class.java, "rightWheel"

            )
        s.direction = Servo.Direction.REVERSE
        s
    }
    val openCloseServo: Servo by OpModeLazyCell {
        val s =
            FeatureRegistrar.activeOpMode.hardwareMap.get(
                Servo::class.java, "openCloseServo"

            )
        s.direction = Servo.Direction.REVERSE
        s
    }
    @JvmField
    var open = 0.59
    @JvmField
    var close = 0.46
    val intake = Lambda("intake")
        .setExecute{
            rightWheel.position = 1.0
            leftWheel.position = 0.0
        }
        .setFinish{false}
        .setEnd{ rightWheel.position = 0.5
            leftWheel.position = 0.5}
    val outtake = Lambda("outtake")
        .setExecute{
            rightWheel.position = 0.0
            leftWheel.position = 1.0
        }
        .setFinish{false}
        .setEnd{ rightWheel.position = 0.5
         leftWheel.position = 0.5}
    val openIntake = Lambda("oi")
        .setInit{ openCloseServo.position = open}
    val closeIntake = Lambda("ci")
        .setInit{ openCloseServo.position = close}
    val changeClawPos = Lambda("ccp")
        .setInit{
            if (openCloseServo.position == open){
                openCloseServo.position = close
            }
            else{
                openCloseServo.position = open
            }
        }
}
