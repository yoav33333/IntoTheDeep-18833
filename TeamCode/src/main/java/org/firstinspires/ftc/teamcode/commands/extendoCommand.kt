package org.firstinspires.ftc.teamcode.commands

import com.acmerobotics.dashboard.config.Config
import dev.frozenmilk.dairy.core.dependency.Dependency
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation
import dev.frozenmilk.dairy.core.wrapper.Wrapper
import dev.frozenmilk.mercurial.Mercurial
import dev.frozenmilk.mercurial.commands.Lambda
import dev.frozenmilk.mercurial.commands.groups.Parallel
import dev.frozenmilk.mercurial.commands.groups.Sequential
import dev.frozenmilk.mercurial.commands.util.IfElse
import dev.frozenmilk.mercurial.commands.util.Wait
import dev.frozenmilk.mercurial.subsystems.Subsystem
import org.firstinspires.ftc.teamcode.subsystems.armClawSubsystem
import org.firstinspires.ftc.teamcode.subsystems.armClawSubsystem.anglePostTransfer
import org.firstinspires.ftc.teamcode.subsystems.armClawSubsystem.angleTransfer
import org.firstinspires.ftc.teamcode.subsystems.clawSubsystem
import org.firstinspires.ftc.teamcode.subsystems.deposit
import org.firstinspires.ftc.teamcode.subsystems.deposit.TransferState
import org.firstinspires.ftc.teamcode.subsystems.deposit.almostArmIn
import org.firstinspires.ftc.teamcode.subsystems.deposit.armMaybeOut
import org.firstinspires.ftc.teamcode.subsystems.deposit.halfArmIn
import org.firstinspires.ftc.teamcode.subsystems.deposit.isSpe
import org.firstinspires.ftc.teamcode.subsystems.deposit.transferSeqAuto
import org.firstinspires.ftc.teamcode.subsystems.extendoSubsystem
import org.firstinspires.ftc.teamcode.subsystems.linearSlides
import org.firstinspires.ftc.teamcode.subsystems.linearSlides.getPose
import org.firstinspires.ftc.teamcode.subsystems.linearSlides.nonBlockRTP
import org.firstinspires.ftc.teamcode.commands.util.InstantCommand
import org.firstinspires.ftc.teamcode.commands.util.RunNonBlocking
import org.firstinspires.ftc.teamcode.commands.util.SuperAdvancing
import org.firstinspires.ftc.teamcode.commands.util.WaitUntil
import java.lang.annotation.Inherited
import kotlin.math.abs
@Config
object extendoCommand : Subsystem {

    override var dependency: Dependency<*> = Subsystem.DEFAULT_DEPENDENCY and
            SingleAnnotation(Mercurial.Attach::class.java)

    @Target(AnnotationTarget.CLASS)
    @Retention(AnnotationRetention.RUNTIME)
    @MustBeDocumented
    @Inherited
    annotation class Attach

    var doTransfer = true
    val noTransfer = Lambda("nt")
        .setInit{ doTransfer = false
        extendoMacro.schedule()}
    val regular = Lambda("r")
        .setInit{ doTransfer = true
            openExtendo = true
        extendoMacro.schedule()}
    var openExtendo = true
    val notOpen = Lambda("no")
        .setInit{ openExtendo = false
        extendoMacro.schedule()}
    var doFlip = true
    val toggleFlip = Lambda("tf")
        .setInit{doFlip = !doFlip}

    override fun preUserInitLoopHook(opMode: Wrapper) {
        doFlip = false
    }
//    val open = Lambda("o")
//        .setInit{ openExtendo = true
//        extendoMacro.schedule()}
    @JvmStatic
    val extendoOpenCommand = Parallel(
        Sequential(
            Parallel(
//                linearSlides.closeSlides,
                halfArmIn,
                deposit.release,
                clawSubsystem.resetAngleClaw,
                armClawSubsystem.openClawArm,
//                IfElse(
//                    extendoSubsystem.extendoOpen,
//                    extendoSubsystem.openExtendo,
//                    InstantCommand{}),
                IfElse({ openExtendo }, extendoSubsystem.openExtendo, extendoSubsystem.openExtendoLil)
//                TransferState,
                ),
                Wait(0.1),
                Parallel(
                    linearSlides.closeSlidesDumb,
                    clawSubsystem.openClaw,
                ),
//                TransferState,
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
                clawSubsystem.openClaw,
                halfArmIn,
                RunNonBlocking(linearSlides.closeSlidesAuto)
            ),
//            Wait(0.2),
            Parallel(
                extendoSubsystem.openExtendo,
            ),
//            TransferState,

        )
    )
    @JvmStatic
    val extendoOpenCommandAutoPush = Parallel(
        Sequential(
            Parallel(
                InstantCommand{clawSubsystem.clawServo.position = 0.02},
//                clawSubsystem.resetAngleClaw,
                armClawSubsystem.extendoPushState,
                clawSubsystem.closeClaw,
                halfArmIn,
                RunNonBlocking(linearSlides.closeSlidesAuto)
            ),
//            Wait(0.2),
            Parallel(
                extendoSubsystem.openExtendo,
            ),
//            TransferState,

        )
    )
    @JvmStatic
    val nonExtendoOpenCommandAuto = Parallel(
        Sequential(
            Parallel(
                clawSubsystem.resetAngleClaw,
                armClawSubsystem.openClawArm,
                clawSubsystem.openClaw,
                halfArmIn,
            ),
            RunNonBlocking(Sequential(
//            Parallel(
                linearSlides.closeSlides,
//            ),
            almostArmIn,))
//            Wait(0.2),

//            TransferState,

        )
    )
    @JvmField
    var openArmAtDelta = 16000
    @JvmStatic
    val extendoReset = Parallel(
//        clawSubsystem.resetAngleClaw,
        extendoSubsystem.closeExtendo,
        armClawSubsystem.closeClawArm,
    )
    val extendoCloseCommand = Parallel(
        Sequential(
            Parallel(
//        clawSubsystem.stopCs,
                IfElse({ doFlip }, clawSubsystem.flippedCenter, clawSubsystem.resetAngleClaw),

//                Wait(0.2),
                extendoSubsystem.closeExtendo,
                deposit.almostArmIn,

                Sequential(
                    armClawSubsystem.closeClawArm,
                    IfElse({ doFlip }, Wait(0.25), Wait(0.1)),

                    TransferState,
//                    armClawSubsystem.moveArmIn,
//                    clawSubsystem.closeClaw2,
                    IfElse(
                        { doTransfer },
                        Sequential(

//                            Wait(0.05),
                            transferSeqAuto,
                            nonBlockRTP,
                            anglePostTransfer,
                            Wait(0.16),
                            angleTransfer,
                            WaitUntil{(abs(Mercurial.gamepad2.rightStickY.state) >0.2 ||(linearSlides.target>500 && abs(linearSlides.target-getPose())< openArmAtDelta) || (isSpe && getPose()>1000) )},
                            armMaybeOut
                        ),
                        Wait(0.0))
                )

            ),

        )
    )
    @JvmStatic
    val extendoCloseCommandAuto =
        Sequential(
            Parallel(
//        clawSubsystem.stopCs,
                clawSubsystem.flippedCenter,

//                Wait(0.2),
                extendoSubsystem.closeExtendo,
                deposit.almostArmIn,

                Sequential(
                    armClawSubsystem.closeClawArm,
                    Wait(0.25),
//                    armClawSubsystem.moveArmIn,
//                    clawSubsystem.closeClaw2,
                    TransferState,
                    Wait(0.15),
                    transferSeqAuto,
                )

            ),
            Parallel(
                linearSlides.nonBlockRTP,
//                Sequential(
//                    anglePostTransfer,
//                    Wait(0.16),
//                    angleTransfer
//                )
            ),
            RunNonBlocking( Sequential(
//                utilCommands.waitUntil{abs(Mercurial.gamepad2.rightStickY.state) >0.2 || isSpe},
                WaitUntil{(abs(Mercurial.gamepad2.rightStickY.state) >0.2 ||(linearSlides.target>500 && linearSlides.target-5000<getPose()) || (isSpe && getPose()>1000) )},
                armMaybeOut
            ))
        )
    @JvmStatic
    val extendoCloseCommandSimple =
        Parallel(
//        clawSubsystem.stopCs,
            clawSubsystem.flippedCenter,
            extendoSubsystem.closeExtendo,
            armClawSubsystem.closeClawArm,
            halfArmIn,
            nonBlockRTP

        )
    val extendoMacro =
        SuperAdvancing(extendoCloseCommand, extendoOpenCommand)

    override fun preUserInitHook(opMode: Wrapper) {
        extendoMacro.reset()
//        extendoMacro.schedule()
    }
}