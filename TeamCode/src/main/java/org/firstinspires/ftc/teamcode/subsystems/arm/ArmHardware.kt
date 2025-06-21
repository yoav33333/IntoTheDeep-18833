package org.firstinspires.ftc.teamcode.subsystems.arm

import com.qualcomm.robotcore.hardware.Servo
import dev.frozenmilk.dairy.core.dependency.Dependency
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation
import dev.frozenmilk.dairy.core.util.OpModeLazyCell
import dev.frozenmilk.dairy.core.wrapper.Wrapper
import dev.frozenmilk.mercurial.Mercurial
import dev.frozenmilk.mercurial.commands.Lambda
import dev.frozenmilk.mercurial.subsystems.SDKSubsystem
import dev.frozenmilk.mercurial.subsystems.Subsystem
import org.firstinspires.ftc.teamcode.util.HardwareDevice
import java.lang.annotation.Inherited
import java.util.function.DoubleSupplier

object ArmHardware: SDKSubsystem(){
    override var dependency: Dependency<*> = Subsystem.DEFAULT_DEPENDENCY and
            SingleAnnotation(Mercurial.Attach::class.java)

    @Target(AnnotationTarget.CLASS)
    @Retention(AnnotationRetention.RUNTIME)
    @MustBeDocumented
    @Inherited
    annotation class Attach
    val armServoRight by OpModeLazyCell{ HardwareDevice("deposit arm r", Servo::class.java).get() }
    val armServoLeft by OpModeLazyCell{ HardwareDevice("deposit arm l", Servo::class.java).get() }
    val extensionServo by OpModeLazyCell{ HardwareDevice("deposit extention", Servo::class.java).get() }

    fun setArmPosition(position: Double) {
        armServoRight.position = position
        armServoLeft.position = position
    }


    fun setArmPositionCommand(position: DoubleSupplier) =
        Lambda("ArmPosition: {}".format(position.asDouble))
            .setInit {
                setArmPosition(position.asDouble)
            }
            .setRunStates(Wrapper.OpModeState.ACTIVE, Wrapper.OpModeState.INIT)
    fun setExtensionPosition(position: Double) {
        extensionServo.position = position
    }

    fun setExtendingArmPosition(armPosition: DoubleSupplier? = null, extensionPosition: DoubleSupplier? = null):Lambda
    = Lambda("ArmPosition: {}, ExtensionPosition: {}".format(armPosition, extensionPosition))
    .setInit{
        if (armPosition !=null){
            setArmPosition(armPosition.asDouble)
        }
        if (extensionPosition !=null){
            setExtensionPosition(extensionPosition.asDouble)
        }
    }.setRunStates(Wrapper.OpModeState.ACTIVE, Wrapper.OpModeState.INIT)
}