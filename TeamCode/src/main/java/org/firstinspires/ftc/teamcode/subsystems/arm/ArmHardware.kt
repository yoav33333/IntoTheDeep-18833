package org.firstinspires.ftc.teamcode.subsystems.arm

import com.qualcomm.robotcore.hardware.Servo
import dev.frozenmilk.dairy.core.dependency.Dependency
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation
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
    val armServo = HardwareDevice("flip servo", Servo::class.java).get()

    /**
     * Sets the arm servo to the specified position.
     *
     * @param position The target servo position, typically between 0.0 (minimum) and 1.0 (maximum).
     */
    fun setArmPosition(position: Double) {
        armServo.position = position
    }


    /**
             * Creates a command that sets the arm servo to the specified position during the INIT and ACTIVE operation modes.
             *
             * @param position The target position for the arm servo.
             * @return A Lambda command that sets the arm position when initialized.
             */
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