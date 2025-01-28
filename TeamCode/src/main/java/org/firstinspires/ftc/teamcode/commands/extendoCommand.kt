package org.firstinspires.ftc.teamcode.commands

import dev.frozenmilk.dairy.core.dependency.Dependency
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation
import dev.frozenmilk.dairy.core.wrapper.Wrapper
import dev.frozenmilk.mercurial.Mercurial
import dev.frozenmilk.mercurial.commands.groups.Parallel
import dev.frozenmilk.mercurial.commands.groups.Sequential
import dev.frozenmilk.mercurial.commands.util.Wait
import dev.frozenmilk.mercurial.subsystems.Subsystem
import org.firstinspires.ftc.teamcode.subsystems.armClawSubsystem
import org.firstinspires.ftc.teamcode.subsystems.armClawSubsystem.anglePostTransfer
import org.firstinspires.ftc.teamcode.subsystems.armClawSubsystem.angleTransfer
import org.firstinspires.ftc.teamcode.subsystems.clawSubsystem
import org.firstinspires.ftc.teamcode.subsystems.deposit.TransferState
import org.firstinspires.ftc.teamcode.subsystems.deposit.transferSeq
import org.firstinspires.ftc.teamcode.subsystems.extendoSubsystem
import org.firstinspires.ftc.teamcode.subsystems.linearSlides
import org.firstinspires.ftc.teamcode.subsystems.linearSlides.nonBlockRTP
import org.firstinspires.ftc.teamcode.util.SuperAdvancing
import java.lang.annotation.Inherited

object extendoCommand : Subsystem {

    override var dependency: Dependency<*> = Subsystem.DEFAULT_DEPENDENCY and
            SingleAnnotation(Mercurial.Attach::class.java)

    @Target(AnnotationTarget.CLASS)
    @Retention(AnnotationRetention.RUNTIME)
    @MustBeDocumented
    @Inherited
    annotation class Attach


    val extendoOpenCommand = Parallel(
        Sequential(
            Parallel(
                linearSlides.closeSlides,
                clawSubsystem.resetAngleClaw,
                armClawSubsystem.openClawArm,
                extendoSubsystem.openExtendo,
                clawSubsystem.openClaw,
                TransferState,
                ),
//            Wait(0.3),
//        clawSubsystem.runCs,
//        antonySubsystem.colorSensorData
        )
    )
    val extendoOpenCommandAuto = Parallel(
        Sequential(
            Parallel(
                linearSlides.closeSlidesAuto,
                clawSubsystem.resetAngleClaw,
                armClawSubsystem.openClawArm,
                extendoSubsystem.openExtendo,
                clawSubsystem.openClaw,
                TransferState,
                ),
//            Wait(0.3),
//        clawSubsystem.runCs,
//        antonySubsystem.colorSensorData
        )
    )
    val extendoReset = Parallel(
        clawSubsystem.resetAngleClaw,
        extendoSubsystem.closeExtendo,
        armClawSubsystem.closeClawArm,
    )
    val extendoCloseCommand = Parallel(
        Sequential(
            Parallel(
//        clawSubsystem.stopCs,
                clawSubsystem.resetAngleClaw,
                extendoSubsystem.closeExtendo,
                armClawSubsystem.closeClawArm,
                TransferState,
                Sequential(
                    Wait(0.3),
                    clawSubsystem.closeClaw2,
                    transferSeq
                )

            ),
            Parallel(
                linearSlides.runToPosition,
                Sequential(
                    Wait(0.2),
                    anglePostTransfer,
                    Wait(0.2),
                    angleTransfer
                )
            )
        )
    )
    val extendoCloseCommandAuto =
        Parallel(
//        clawSubsystem.stopCs,
            clawSubsystem.resetAngleClaw,
            extendoSubsystem.closeExtendo,
            armClawSubsystem.closeClawArm,
            TransferState,
            Sequential(
                Wait(0.3),
                clawSubsystem.closeClaw2,
                transferSeq,
                nonBlockRTP
            )

        )
    val extendoMacro =
        SuperAdvancing(extendoCloseCommand, extendoOpenCommand)

    override fun preUserInitHook(opMode: Wrapper) {
        extendoMacro.restart()
//        extendoMacro.schedule()
    }
}