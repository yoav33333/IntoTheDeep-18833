package org.firstinspires.ftc.teamcode.subsystems.robot

import dev.frozenmilk.mercurial.Mercurial
import dev.frozenmilk.mercurial.commands.Lambda
import dev.frozenmilk.mercurial.commands.groups.Parallel
import dev.frozenmilk.mercurial.commands.groups.Sequential
import dev.frozenmilk.mercurial.commands.util.IfElse
import dev.frozenmilk.mercurial.commands.util.Wait
import org.firstinspires.ftc.teamcode.commands.extendoCommand
import org.firstinspires.ftc.teamcode.commands.extendoCommand.doFlip
import org.firstinspires.ftc.teamcode.commands.extendoCommand.openArmAtDelta
import org.firstinspires.ftc.teamcode.commands.util.RunNonBlocking
import org.firstinspires.ftc.teamcode.commands.util.SuperAdvancing
import org.firstinspires.ftc.teamcode.commands.util.WaitUntil
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmCommands.avoidBasket
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmCommands.moveToSlam
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmCommands.moveToTransfer
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmCommands.smartDeposit
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmVariables.SlamArmPosition
import org.firstinspires.ftc.teamcode.subsystems.armClawSubsystem
import org.firstinspires.ftc.teamcode.subsystems.armClawSubsystem.anglePostTransfer
import org.firstinspires.ftc.teamcode.subsystems.armClawSubsystem.angleTransfer
import org.firstinspires.ftc.teamcode.subsystems.clawSubsystem
import org.firstinspires.ftc.teamcode.subsystems.deposit
import org.firstinspires.ftc.teamcode.subsystems.deposit.armMaybeOut
import org.firstinspires.ftc.teamcode.subsystems.deposit.catchPixel
import org.firstinspires.ftc.teamcode.subsystems.deposit.intakeCommand
import org.firstinspires.ftc.teamcode.subsystems.deposit.isSpe
import org.firstinspires.ftc.teamcode.subsystems.deposit.postIntakeState
import org.firstinspires.ftc.teamcode.subsystems.deposit.release
import org.firstinspires.ftc.teamcode.subsystems.deposit.slamArm
import org.firstinspires.ftc.teamcode.subsystems.deposit.transferSeqAuto
import org.firstinspires.ftc.teamcode.subsystems.deposit.up
import org.firstinspires.ftc.teamcode.subsystems.depositClaw.DepositClawCommands.closeDepositClaw
import org.firstinspires.ftc.teamcode.subsystems.depositClaw.DepositClawCommands.closeWhenSampleInPlace
import org.firstinspires.ftc.teamcode.subsystems.depositClaw.DepositClawCommands.openDepositClaw
import org.firstinspires.ftc.teamcode.subsystems.extendo.ExtendoCommands.closeExtendo
import org.firstinspires.ftc.teamcode.subsystems.extendo.ExtendoCommands.openExtendo

import org.firstinspires.ftc.teamcode.subsystems.extendo.ExtendoCommands.partialOpenExtendo
import org.firstinspires.ftc.teamcode.subsystems.extendo.ExtendoState
import org.firstinspires.ftc.teamcode.subsystems.extendo.ExtendoVariables.extendoState
import org.firstinspires.ftc.teamcode.subsystems.extendoSubsystem
import org.firstinspires.ftc.teamcode.subsystems.intakeClaw.IntakeClawCommands.openIntakeClaw
import org.firstinspires.ftc.teamcode.subsystems.intakeClaw.IntakeClawCommands.resetAngleClaw
import org.firstinspires.ftc.teamcode.subsystems.lift.LiftCommands.closeSlides
import org.firstinspires.ftc.teamcode.subsystems.lift.LiftCommands.down
import org.firstinspires.ftc.teamcode.subsystems.lift.LiftCommands.enablePID
import org.firstinspires.ftc.teamcode.subsystems.lift.LiftHardware
import org.firstinspires.ftc.teamcode.subsystems.lift.LiftVariables
import org.firstinspires.ftc.teamcode.subsystems.linearSlides
import org.firstinspires.ftc.teamcode.subsystems.linearSlides.getPose
import org.firstinspires.ftc.teamcode.subsystems.linearSlides.nonBlockRTP
import org.firstinspires.ftc.teamcode.subsystems.robot.RobotVariables.transferState

import org.firstinspires.ftc.teamcode.subsystems.v4b.V4bCommands.closeIntakeArm
import org.firstinspires.ftc.teamcode.subsystems.v4b.V4bCommands.openIntakeArm
import org.firstinspires.ftc.teamcode.subsystems.v4b.V4bCommands.postTransferSequence
import kotlin.math.abs

object RobotCommands {
    val doTransfer = Lambda("doTransfer")
        .setInit{transferState = TransferState.TRANSFER}

    val doNothing = Lambda("doNothing")
        .setInit{transferState = TransferState.DO_NOTHING}

    val openCommand =
        Sequential(
            Parallel(
                avoidBasket,
                resetAngleClaw,
                openIntakeArm,
                IfElse({ extendoState == ExtendoState.FULL_OPEN }, openExtendo, partialOpenExtendo)
            ),
            Wait(0.1),
            Parallel(
                closeSlides,
                openIntakeClaw,
            )
        )

    val openCommandAuto =
        Sequential(
            Parallel(
                openIntakeArm,
                openIntakeClaw,
                avoidBasket,
                RunNonBlocking(closeSlides)
            ),
            openExtendo
        )

    val reset = Parallel(
        closeExtendo,
        closeIntakeArm,
    )

    val closeCommand =
        Sequential(
            resetAngleClaw,
            closeExtendo,
            moveToTransfer,
            closeIntakeArm,
            openDepositClaw,
            Sequential(
                IfElse(
                    { transferState == TransferState.TRANSFER },
                    Sequential(
                        closeWhenSampleInPlace.raceWith(Wait(1.0)),
                        closeDepositClaw,
                        enablePID,
                        postTransferSequence,
                        WaitUntil{(abs(Mercurial.gamepad2.rightStickY.state) >0.2 ||(LiftVariables.targetPosition>500 && abs(
                            LiftVariables.targetPosition- LiftHardware.getPose()
                        ) < RobotVariables.deltaToReopenAfterSwitch) || (RobotVariables.gameElement == GameElement.SPECIMEN && LiftHardware.getPose() >1000) )},
                        smartDeposit
                    ),
                    Wait(0.0))
            )

        )

    val macro = SuperAdvancing(closeCommand, openCommand)

    @JvmStatic
    val slamSeq = Sequential(moveToSlam, Wait(0.0), down, Wait(0.15), openDepositClaw)

}