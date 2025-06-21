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
import java.util.function.DoubleSupplier

object V4bHardware: SDKSubsystem() {
    override var dependency: Dependency<*> = Subsystem.DEFAULT_DEPENDENCY and
            SingleAnnotation(Mercurial.Attach::class.java)

    @Target(AnnotationTarget.CLASS)
    @Retention(AnnotationRetention.RUNTIME)
    @MustBeDocumented
    @Inherited
    annotation class Attach

    val armServo by OpModeLazyCell{ HardwareDevice("4 bar", Servo::class.java).get() }
    val pitchServo by OpModeLazyCell{ HardwareDevice("arm", Servo::class.java).get()}

    fun setArmPosition(position: DoubleSupplier) {
        armServo.position = position.asDouble
    }

    fun getArmPosition(): Double {
        return armServo.position
    }

    fun setPitchPosition(position: DoubleSupplier) {
        pitchServo.position = position.asDouble
    }

    fun getPitchPosition(): Double {
        return pitchServo.position
    }

    override fun preUserInitHook(opMode: Wrapper) {
        armOutPosition =  if (activeOpModeWrapper.opModeType == OpModeMeta.Flavor.TELEOP) armOutPositionTele  else armOutPositionAuto
    }

}