package org.firstinspires.ftc.teamcode.subsystems.intakeClaw

import com.qualcomm.robotcore.hardware.Servo
import dev.frozenmilk.dairy.core.dependency.Dependency
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation
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

    val intakeClaw = HardwareDevice("intake claw", Servo::class.java).get()
    val rotationServo = HardwareDevice("rotate servo", Servo::class.java).get()

    /**
     * Sets the position of the intake claw servo.
     *
     * @param position The desired servo position, typically between 0.0 (minimum) and 1.0 (maximum).
     */
    fun setIntakeClawPosition(position: Double) {
        intakeClaw.position = position
    }

    /**
     * Returns the current position of the intake claw servo.
     *
     * @return The position value of the intake claw servo.
     */
    fun getIntakeClawPosition(): Double {
        return intakeClaw.position
    }

    /**
     * Sets the position of the rotation servo.
     *
     * @param position The desired servo position, typically between 0.0 and 1.0.
     */
    fun setRotationPosition(position: Double) {
        rotationServo.position = position
    }

    /**
     * Returns the current position of the rotation servo.
     *
     * @return The position of the rotation servo as a value between 0.0 and 1.0.
     */
    fun getRotationPosition(): Double {
        return rotationServo.position
    }

}