package org.firstinspires.ftc.teamcode.subsystems.v4b

import com.qualcomm.robotcore.hardware.Servo
import dev.frozenmilk.dairy.core.FeatureRegistrar
import dev.frozenmilk.dairy.core.FeatureRegistrar.activeOpModeWrapper
import dev.frozenmilk.dairy.core.dependency.Dependency
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation
import dev.frozenmilk.dairy.core.util.OpModeLazyCell
import dev.frozenmilk.dairy.core.wrapper.Wrapper
import dev.frozenmilk.mercurial.Mercurial
import dev.frozenmilk.mercurial.subsystems.SDKSubsystem
import dev.frozenmilk.mercurial.subsystems.Subsystem
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta
import org.firstinspires.ftc.teamcode.subsystems.v4b.V4bVariables.armOutPosition
import org.firstinspires.ftc.teamcode.subsystems.v4b.V4bVariables.armOutPositionAuto
import org.firstinspires.ftc.teamcode.subsystems.v4b.V4bVariables.armOutPositionTele
import org.firstinspires.ftc.teamcode.util.HardwareDevice
import java.lang.annotation.Inherited

object V4bHardware: SDKSubsystem() {
    override var dependency: Dependency<*> = Subsystem.DEFAULT_DEPENDENCY and
            SingleAnnotation(Mercurial.Attach::class.java)

    @Target(AnnotationTarget.CLASS)
    @Retention(AnnotationRetention.RUNTIME)
    @MustBeDocumented
    @Inherited
    annotation class Attach

    val armServo = HardwareDevice("4 bar s", Servo::class.java).get()
    val pitchServo = HardwareDevice("arm s", Servo::class.java).get()

    /**
     * Sets the position of the arm servo to the specified value.
     *
     * @param position The desired servo position, typically between 0.0 and 1.0.
     */
    fun setArmPosition(position: Double) {
        armServo.position = position
    }

    /**
     * Returns the current position of the arm servo.
     *
     * @return The position of the arm servo as a value between 0.0 and 1.0.
     */
    fun getArmPosition(): Double {
        return armServo.position
    }

    /**
     * Sets the position of the pitch servo controlling the four-bar linkage.
     *
     * @param position The target servo position, typically between 0.0 (minimum) and 1.0 (maximum).
     */
    fun setPitchPosition(position: Double) {
        pitchServo.position = position
    }

    /**
     * Returns the current position of the pitch servo.
     *
     * @return The position of the pitch servo as a value between 0.0 and 1.0.
     */
    fun getPitchPosition(): Double {
        return pitchServo.position
    }

    /**
     * Sets the global arm out position preset based on the current operation mode flavor.
     *
     * If the active operation mode is TeleOp, assigns the TeleOp preset; otherwise, assigns the Autonomous preset.
     */
    override fun preUserInitHook(opMode: Wrapper) {
        armOutPosition =  if (activeOpModeWrapper.opModeType == OpModeMeta.Flavor.TELEOP) armOutPositionTele  else armOutPositionAuto
    }

}