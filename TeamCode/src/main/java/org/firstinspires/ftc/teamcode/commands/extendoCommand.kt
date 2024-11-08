package org.firstinspires.ftc.teamcode.commands

import dev.frozenmilk.dairy.core.dependency.Dependency
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation
import dev.frozenmilk.dairy.core.wrapper.Wrapper
import dev.frozenmilk.mercurial.Mercurial
import dev.frozenmilk.mercurial.commands.Lambda
import dev.frozenmilk.mercurial.commands.groups.Parallel
import dev.frozenmilk.mercurial.subsystems.Subsystem
import dev.frozenmilk.util.cell.RefCell
import org.firstinspires.ftc.teamcode.subsystems.armClawSubsystem
import org.firstinspires.ftc.teamcode.subsystems.clawSubsystem
import org.firstinspires.ftc.teamcode.subsystems.extendoSubsystem
import java.lang.annotation.Inherited

object extendoCommand : Subsystem{
    enum class extendoState{
        OPEN,
        CLOSE;
    }
    val currentExtendoState: RefCell<extendoState> = RefCell(extendoState.CLOSE)

    override var dependency: Dependency<*> = Subsystem.DEFAULT_DEPENDENCY and
            SingleAnnotation(Mercurial.Attach::class.java)
    @Target(AnnotationTarget.CLASS)
    @Retention(AnnotationRetention.RUNTIME)
    @MustBeDocumented
    @Inherited
    annotation class Attach
    fun extendoCloseCommand(){

        clawSubsystem.clawRotationServo.position = 0.0
        clawSubsystem.closeClaw()
        armClawSubsystem.closeClawArm()
        extendoSubsystem.closeExtendoF()

    }
    fun extendoOpenCommand(){
        clawSubsystem.clawRotationServo.position = 0.0
        clawSubsystem.openClaw()
        armClawSubsystem.openClawArm
        extendoSubsystem.openExtendoF()
    }
    val changeExtendoState = Lambda("changeExtendoState")
        .setInit{
            if (currentExtendoState.get() == extendoState.OPEN){
                extendoCloseCommand()
                currentExtendoState.accept(extendoState.CLOSE)
            }
            else{
                extendoOpenCommand()
                currentExtendoState.accept(extendoState.OPEN)
            }

        }.setRunStates(Wrapper.OpModeState.ACTIVE)
        .setRequirements(Mercurial, clawSubsystem, extendoSubsystem)
}