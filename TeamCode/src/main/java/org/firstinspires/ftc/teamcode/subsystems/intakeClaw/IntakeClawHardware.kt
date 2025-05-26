package org.firstinspires.ftc.teamcode.subsystems.intakeClaw

import com.qualcomm.robotcore.hardware.Servo
import dev.frozenmilk.dairy.core.dependency.Dependency
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation
import dev.frozenmilk.dairy.core.util.OpModeLazyCell
import dev.frozenmilk.mercurial.Mercurial
import dev.frozenmilk.mercurial.subsystems.SDKSubsystem
import dev.frozenmilk.mercurial.subsystems.Subsystem
import org.firstinspires.ftc.teamcode.util.HardwareDevice
import java.lang.annotation.Inherited

object IntakeClawHardware: SDKSubsystem() {
    override var dependency: Dependency<*> = Subsystem.DEFAULT_DEPENDENCY and
            SingleAnnotation(Mercurial.Attach::class.java)

    @Target(AnnotationTarget.CLASS)
    @Retention(AnnotationRetention.RUNTIME)
    @MustBeDocumented
    @Inherited
    annotation class Attach

    val intakeClaw by OpModeLazyCell{HardwareDevice("claw servo", Servo::class.java).get()}
    val rotationServo by OpModeLazyCell{HardwareDevice("rotate servo", Servo::class.java).get()}

    fun setIntakeClawPosition(position: Double) {
        intakeClaw.position = position
    }

    fun getIntakeClawPosition(): Double {
        return intakeClaw.position
    }

    fun setRotationPosition(position: Double) {
        rotationServo.position = position
    }

    fun getRotationPosition(): Double {
        return rotationServo.position
    }

}