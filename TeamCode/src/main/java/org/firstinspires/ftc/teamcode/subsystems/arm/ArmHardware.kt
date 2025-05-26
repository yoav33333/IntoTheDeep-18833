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

object ArmHardware: SDKSubsystem(){
    override var dependency: Dependency<*> = Subsystem.DEFAULT_DEPENDENCY and
            SingleAnnotation(Mercurial.Attach::class.java)

    @Target(AnnotationTarget.CLASS)
    @Retention(AnnotationRetention.RUNTIME)
    @MustBeDocumented
    @Inherited
    annotation class Attach
    val armServo by OpModeLazyCell{ HardwareDevice("flip servo", Servo::class.java).get() }

    fun setArmPosition(position: Double) {
        armServo.position = position
    }


    fun setArmPositionCommand(position: Double) =
        Lambda("ArmPosition: {}".format(position))
            .setInit {
                setArmPosition(position)
            }
            .setRunStates(Wrapper.OpModeState.ACTIVE, Wrapper.OpModeState.INIT)
//    fun setExtensionPosition(position: Double) {
//        extensionServo.position = position
//    }

//    fun setExtendingArmPosition(armPosition: Double? = null, extensionPosition: Double? = null):Lambda
//    = Lambda("ExtendingArmPosition")
//    .setInit{
//        if (armPosition !=null){
//            setArmPosition(armPosition)
//        }
//        if (extensionPosition !=null){
//            setExtensionPosition(extensionPosition)
//        }
//    }.setRunStates(Wrapper.OpModeState.ACTIVE, Wrapper.OpModeState.INIT)
}