package org.firstinspires.ftc.teamcode.commands

import dev.frozenmilk.dairy.core.Feature
import dev.frozenmilk.dairy.core.dependency.Dependency
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation
import dev.frozenmilk.dairy.core.util.OpModeLazyCell
import dev.frozenmilk.dairy.core.wrapper.Wrapper
import dev.frozenmilk.mercurial.Mercurial
import dev.frozenmilk.mercurial.commands.Lambda
import dev.frozenmilk.mercurial.commands.groups.Advancing
import dev.frozenmilk.mercurial.commands.groups.Parallel
import dev.frozenmilk.mercurial.commands.groups.Sequential
import dev.frozenmilk.mercurial.commands.util.IfElse
import dev.frozenmilk.mercurial.commands.util.StateMachine
import dev.frozenmilk.mercurial.commands.util.Wait
import dev.frozenmilk.mercurial.subsystems.Subsystem
import dev.frozenmilk.util.cell.LazyCell
import dev.frozenmilk.util.cell.RefCell
import org.firstinspires.ftc.teamcode.subsystems.antonySubsystem
import org.firstinspires.ftc.teamcode.subsystems.armClawSubsystem
import org.firstinspires.ftc.teamcode.subsystems.armClawSubsystem.anglePostTransfer
import org.firstinspires.ftc.teamcode.subsystems.armClawSubsystem.angleTransfer
import org.firstinspires.ftc.teamcode.subsystems.clawSubsystem
import org.firstinspires.ftc.teamcode.subsystems.deposit.TransferState
import org.firstinspires.ftc.teamcode.subsystems.deposit.armIn
import org.firstinspires.ftc.teamcode.subsystems.deposit.transferCommand
import org.firstinspires.ftc.teamcode.subsystems.deposit.transferSeq
import org.firstinspires.ftc.teamcode.subsystems.extendoSubsystem
import org.firstinspires.ftc.teamcode.subsystems.linearSlides
import org.firstinspires.ftc.teamcode.util.SuperAdvancing
import java.lang.annotation.Inherited
import kotlin.properties.Delegates

object extendoCommand : Subsystem{

    override var dependency: Dependency<*> = Subsystem.DEFAULT_DEPENDENCY and
            SingleAnnotation(Mercurial.Attach::class.java)

    @Target(AnnotationTarget.CLASS)
    @Retention(AnnotationRetention.RUNTIME)
    @MustBeDocumented
    @Inherited
    annotation class Attach



    val extendoOpenCommand = Parallel(Sequential(Parallel(
        linearSlides.closeSlides,
        clawSubsystem.resetAngleClaw,
        armClawSubsystem.openClawArm,
        extendoSubsystem.openExtendo,
        clawSubsystem.openClaw,
    ),Wait(0.3),
//        clawSubsystem.runCs,
        TransferState,
//        antonySubsystem.colorSensorData
    ))
    val extendoCloseCommand = Parallel(Sequential(Parallel(
//        clawSubsystem.stopCs,
        clawSubsystem.closeClaw2,
        clawSubsystem.resetAngleClaw,
        extendoSubsystem.closeExtendo,
        armClawSubsystem.closeClawArm,
        TransferState,
        transferSeq
    ),
        linearSlides.runToPosition,
        Wait(0.2),
        anglePostTransfer,
        Wait(0.2),
        angleTransfer,
    ))

    val extendoMacro =
        SuperAdvancing(extendoCloseCommand, extendoOpenCommand)

    override fun preUserStartHook(opMode: Wrapper) {
        extendoMacro.restart()
        extendoMacro.schedule()
    }
}