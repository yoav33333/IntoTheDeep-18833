package org.firstinspires.ftc.teamcode.subsystems


import com.acmerobotics.dashboard.config.Config
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

@Config
object extendoSubsystem : Subsystem {
    override var dependency: Dependency<*> = Subsystem.DEFAULT_DEPENDENCY and
            SingleAnnotation(Mercurial.Attach::class.java)

    @Target(AnnotationTarget.CLASS)
    @Retention(AnnotationRetention.RUNTIME)
    @MustBeDocumented
    @Inherited
    annotation class Attach

    val extendoServoL: CachingServo by OpModeLazyCell {
        val s = CachingServo(
            FeatureRegistrar.activeOpMode.hardwareMap.get(
                Servo::class.java, "extendo l s"
            )
        )
        s
    }
    val extendoServoR: CachingServo by OpModeLazyCell {
        val s = CachingServo(
            FeatureRegistrar.activeOpMode.hardwareMap.get(
                Servo::class.java, "extendo r s"
            )
        )
        s
    }
    val open = 0.33
    @JvmField
    var lilOpen = 0.75
    val close = 0.99
    var extendoOpen = true

    /**
     * Resets the extendo open state to true before user initialization.
     *
     * Called during the subsystem's pre-user initialization phase to ensure the extendo mechanism starts in the open state.
     */
    override fun preUserInitHook(opMode: Wrapper) {
        extendoOpen = true
    }
    /**
     * Sets both extendo servos to the open position.
     */
    fun openExtendoF() {
        extendoServoR.setPosition(open)
        extendoServoL.setPosition(open)
    }

    /**
     * Sets both extendo servos to the closed position.
     */
    fun closeExtendoF() {
        extendoServoR.setPosition(close)
        extendoServoL.setPosition(close)
    }
    val changeOpenExtendo = Lambda("coe")
        .setInit{ extendoOpen = !extendoOpen}
    val moveManualO = Lambda("moveManualO")
        .setRunStates(Wrapper.OpModeState.ACTIVE)
        .setExecute {
            extendoServoR.setPosition(if (extendoServoR.position < 1) extendoServoR.position + 0.01 else extendoServoR.position)
            extendoServoL.setPosition(if (extendoServoL.position < 1) extendoServoL.position + 0.01 else extendoServoL.position)
        }
    val moveManual = Lambda("moveManual")
        .setRunStates(Wrapper.OpModeState.ACTIVE)
        .setExecute {
//            extendoServoR.position+=(Mercurial.gamepad2.leftStickY.state/100.0)
//            extendoServoL.position+=(Mercurial.gamepad2.leftStickY.state/100.0)
            if (extendoServoL.position - Mercurial.gamepad2.leftStickY.state / 50.0 > 0.3) {
                extendoServoL.position -= Mercurial.gamepad2.leftStickY.state / 50.0
                extendoServoR.position -= Mercurial.gamepad2.leftStickY.state / 50.0
            }
        }
        .setFinish { false }
    val moveManualC = Lambda("moveManualC")
        .setRunStates(Wrapper.OpModeState.ACTIVE)
        .setExecute {
            extendoServoR.setPosition(if (extendoServoR.position > 0.2) extendoServoR.position - 0.01 else extendoServoR.position)
            extendoServoL.setPosition(if (extendoServoL.position > 0.2) extendoServoL.position - 0.01 else extendoServoL.position)
        }

    val openExtendo = Lambda("openExtendo")
        .setInit {
//            if (extendoOpen){
            openExtendoF()
//            }
        }
    val openExtendoLil = Lambda("openExtendo")
        .setInit {
//            if (extendoOpen){
            extendoServoR.setPosition(lilOpen)
            extendoServoL.setPosition(lilOpen)
//            }
        }
    val closeExtendo = Lambda("closeClawArm")
        .setInit {
            closeExtendoF()
        }
}