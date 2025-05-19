package org.firstinspires.ftc.teamcode.subsystems.extendo

import com.qualcomm.robotcore.hardware.Servo
import dev.frozenmilk.dairy.core.dependency.Dependency
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation
import dev.frozenmilk.dairy.core.util.OpModeLazyCell
import dev.frozenmilk.mercurial.Mercurial
import dev.frozenmilk.mercurial.subsystems.SDKSubsystem
import dev.frozenmilk.mercurial.subsystems.Subsystem
import org.firstinspires.ftc.teamcode.util.BetterServo
import java.lang.annotation.Inherited

object ExtendoHardware: SDKSubsystem() {
    override var dependency: Dependency<*> = Subsystem.DEFAULT_DEPENDENCY and
            SingleAnnotation(Mercurial.Attach::class.java)

    @Target(AnnotationTarget.CLASS)
    @Retention(AnnotationRetention.RUNTIME)
    @MustBeDocumented
    @Inherited
    annotation class Attach

    val extendoServoL: Servo by OpModeLazyCell {
        BetterServo("extendo l s")
    }
    val extendoServoR: Servo by OpModeLazyCell {
        BetterServo("extendo r s")
    }

    fun setPosition(position: Double) {
        extendoServoL.position = position
        extendoServoR.position = position
    }


}