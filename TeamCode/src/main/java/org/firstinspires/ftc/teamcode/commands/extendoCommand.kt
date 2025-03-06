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
//import org.firstinspires.ftc.teamcode.subsystems.clawSubsystem
import org.firstinspires.ftc.teamcode.subsystems.deposit
import org.firstinspires.ftc.teamcode.subsystems.deposit.TransferState
import org.firstinspires.ftc.teamcode.subsystems.deposit.armOut
import org.firstinspires.ftc.teamcode.subsystems.deposit.halfArmIn
import org.firstinspires.ftc.teamcode.subsystems.deposit.isSpe
import org.firstinspires.ftc.teamcode.subsystems.deposit.transferSeq
import org.firstinspires.ftc.teamcode.subsystems.deposit.transferSeqAuto
import org.firstinspires.ftc.teamcode.subsystems.extendoSubsystem
import org.firstinspires.ftc.teamcode.subsystems.intakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.linearSlides
import org.firstinspires.ftc.teamcode.subsystems.linearSlides.getPose
import org.firstinspires.ftc.teamcode.subsystems.linearSlides.nonBlockRTP
import org.firstinspires.ftc.teamcode.util.RunNonBlocking
import org.firstinspires.ftc.teamcode.util.SuperAdvancing
import org.firstinspires.ftc.teamcode.util.WaitUntil
import org.firstinspires.ftc.teamcode.util.utilCommands
import java.lang.annotation.Inherited
import kotlin.math.abs

object extendoCommand : Subsystem {

    override var dependency: Dependency<*> = Subsystem.DEFAULT_DEPENDENCY and
            SingleAnnotation(Mercurial.Attach::class.java)

    @Target(AnnotationTarget.CLASS)
    @Retention(AnnotationRetention.RUNTIME)
    @MustBeDocumented
    @Inherited
    annotation class Attach

    @JvmStatic
    val extendoOpenCommand = Parallel(
        Sequential(
            Parallel(
//                linearSlides.closeSlides,
                halfArmIn,
                deposit.release,
//                clawSubsystem.resetAngleClaw,
                armClawSubsystem.openClawArm,
                extendoSubsystem.openExtendo,
                intakeSubsystem.openIntake,
//                TransferState,
                ),
                Wait(0.1),
                Parallel(
                    linearSlides.closeSlides,
                ),
                TransferState,
//            Wait(0.3),
//        clawSubsystem.runCs,
//        antonySubsystem.colorSensorData
        )
    )
    @JvmStatic
    val extendoOpenCommandAuto = Parallel(
        Sequential(
            Parallel(
//                clawSubsystem.resetAngleClaw,
                armClawSubsystem.openClawArm,
                intakeSubsystem.openIntake,
                halfArmIn
            ),
            Wait(0.1),
            Parallel(
                linearSlides.closeSlidesAuto,
                extendoSubsystem.openExtendo,
            ).raceWith(Wait(1.5)),
            TransferState,

        )
    )
    @JvmStatic
    val extendoReset = Parallel(
//        clawSubsystem.resetAngleClaw,
        extendoSubsystem.closeExtendo,
        armClawSubsystem.closeClawArm,
    )
    val extendoCloseCommand = Parallel(

        Sequential(
            intakeSubsystem.closeIntakeFull,
            Wait(0.05),
            Parallel(
//        clawSubsystem.stopCs,
//                clawSubsystem.resetAngleClaw,
                armClawSubsystem.closeClawArm,
                Wait(0.2),
                extendoSubsystem.closeExtendo,
                TransferState,
                Sequential(
                    Wait(0.3),
//                    armClawSubsystem.moveArmIn,
//                    clawSubsystem.closeClaw2,
                    transferSeq
                )

            ),
            Parallel(
                linearSlides.nonBlockRTP,
                Sequential(
                    anglePostTransfer,
                    Wait(0.2),
                    angleTransfer
                )
            ),
            Sequential(
//                utilCommands.waitUntil{abs(Mercurial.gamepad2.rightStickY.state) >0.2 || isSpe},
                WaitUntil{(abs(Mercurial.gamepad2.rightStickY.state) >0.2 ||(linearSlides.target>500 && linearSlides.target-600<getPose()) )},
                armOut
            )
        )
    )
    @JvmStatic
    val extendoCloseCommandAuto =
        Parallel(
//        clawSubsystem.stopCs,
//            clawSubsystem.resetAngleClaw,
            extendoSubsystem.closeExtendo,
            armClawSubsystem.closeClawArm,
            TransferState,
            Sequential(
                Wait(0.3),
                armClawSubsystem.moveArmIn,
//                clawSubsystem.closeClaw2,
                transferSeqAuto,
                nonBlockRTP,
                RunNonBlocking(
                    Sequential(
                        anglePostTransfer,
                        Wait(0.2),
                        angleTransfer
                    )
                ),
                RunNonBlocking(
                    Sequential(
                        WaitUntil{(linearSlides.target>500 && linearSlides.target-300<getPose()) },
                        armOut
                    )
                )
            )

        )
    @JvmStatic
    val extendoCloseCommandSimple =
        Parallel(
//        clawSubsystem.stopCs,
//            clawSubsystem.resetAngleClaw,
            extendoSubsystem.closeExtendo,
            armClawSubsystem.closeClawArm,
            TransferState,
            nonBlockRTP

        )
    val extendoMacro =
        SuperAdvancing(extendoCloseCommand, extendoOpenCommand)

    override fun preUserInitHook(opMode: Wrapper) {
        extendoMacro.restart()
//        extendoMacro.schedule()
    }
}