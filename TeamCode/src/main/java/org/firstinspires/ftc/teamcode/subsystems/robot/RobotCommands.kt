package org.firstinspires.ftc.teamcode.subsystems.robot

import dev.frozenmilk.mercurial.Mercurial
import dev.frozenmilk.mercurial.commands.Lambda
import dev.frozenmilk.mercurial.commands.groups.Parallel
import dev.frozenmilk.mercurial.commands.groups.Sequential
import dev.frozenmilk.mercurial.commands.util.IfElse
import dev.frozenmilk.mercurial.commands.util.Wait
import org.firstinspires.ftc.teamcode.commands.util.InstantCommand
import org.firstinspires.ftc.teamcode.commands.util.RunNonBlocking
import org.firstinspires.ftc.teamcode.commands.util.SuperAdvancing
import org.firstinspires.ftc.teamcode.commands.util.WaitUntil
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmCommands.avoidBasket
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmCommands.closeExtension
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmCommands.moveArmToTransfer
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmCommands.moveToChamber
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmCommands.moveToTransfer
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmCommands.moveToWall
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmCommands.openExtension
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmCommands.smartDeposit
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmHardware.armServoLeft
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmVariables.wallArmPosition
import org.firstinspires.ftc.teamcode.subsystems.depositClaw.DepositClawCommands
import org.firstinspires.ftc.teamcode.subsystems.depositClaw.DepositClawCommands.closeDepositClaw
import org.firstinspires.ftc.teamcode.subsystems.depositClaw.DepositClawCommands.closeWhenSampleInPlace
import org.firstinspires.ftc.teamcode.subsystems.depositClaw.DepositClawCommands.fullyCloseDepositClaw
import org.firstinspires.ftc.teamcode.subsystems.depositClaw.DepositClawCommands.openDepositClaw
import org.firstinspires.ftc.teamcode.subsystems.depositClaw.DepositClawCommands.quickRC
import org.firstinspires.ftc.teamcode.subsystems.extendo.ExtendoCommands.closeExtendo
import org.firstinspires.ftc.teamcode.subsystems.extendo.ExtendoCommands.openExtendo

import org.firstinspires.ftc.teamcode.subsystems.extendo.ExtendoCommands.partialOpenExtendo
import org.firstinspires.ftc.teamcode.subsystems.extendo.ExtendoCommands.setFullOpen
import org.firstinspires.ftc.teamcode.subsystems.extendo.ExtendoCommands.setPartialOpen
import org.firstinspires.ftc.teamcode.subsystems.extendo.ExtendoState
import org.firstinspires.ftc.teamcode.subsystems.extendo.ExtendoVariables.extendoState
import org.firstinspires.ftc.teamcode.subsystems.intakeClaw.IntakeClawCommands.flip
import org.firstinspires.ftc.teamcode.subsystems.intakeClaw.IntakeClawCommands.openIntakeClaw
import org.firstinspires.ftc.teamcode.subsystems.intakeClaw.IntakeClawCommands.resetAngleClaw
import org.firstinspires.ftc.teamcode.subsystems.lift.LiftCommands.closeSlides
//import org.firstinspires.ftc.teamcode.subsystems.lift.LiftCommands.down
import org.firstinspires.ftc.teamcode.subsystems.lift.LiftCommands.enablePID
import org.firstinspires.ftc.teamcode.subsystems.lift.LiftCommands.goToHighChamber
import org.firstinspires.ftc.teamcode.subsystems.lift.LiftCommands.goToWall
import org.firstinspires.ftc.teamcode.subsystems.lift.LiftHardware
import org.firstinspires.ftc.teamcode.subsystems.lift.LiftHardware.getPose
import org.firstinspires.ftc.teamcode.subsystems.lift.LiftVariables
import org.firstinspires.ftc.teamcode.subsystems.lift.LiftVariables.closedPose
import org.firstinspires.ftc.teamcode.subsystems.lift.LiftVariables.targetPosition
import org.firstinspires.ftc.teamcode.subsystems.robot.RobotVariables.transferState
import org.firstinspires.ftc.teamcode.subsystems.v4b.V4bCommands.armUp

import org.firstinspires.ftc.teamcode.subsystems.v4b.V4bCommands.closeIntakeArm
import org.firstinspires.ftc.teamcode.subsystems.v4b.V4bCommands.openIntakeArm
//import org.firstinspires.ftc.teamcode.subsystems.v4b.V4bCommands.pitchWall
import org.firstinspires.ftc.teamcode.subsystems.v4b.V4bCommands.v4bWall
import kotlin.math.abs

object RobotCommands {
    val doTransfer = Lambda("doTransfer")
        .setInit{transferState = TransferState.TRANSFER}

    val doNothing = Lambda("doNothing")
        .setInit{transferState = TransferState.DO_NOTHING}
    @JvmStatic
    val openCommand =
        Sequential(
            Parallel(
                InstantCommand{RobotVariables.extendo = "open"},
                avoidBasket,
                resetAngleClaw,
                openIntakeArm,
                RunNonBlocking(closeSlides),
                IfElse({ extendoState == ExtendoState.FULL_OPEN }, openExtendo, partialOpenExtendo)
            ),
            Wait(0.1),
            Parallel(
                openIntakeClaw,
            ),
            openDepositClaw,
            WaitUntil{getPose()<(closedPose*1.5)},
            moveToTransfer,
            Wait(0.3),
            closeExtension
        )
    @JvmStatic
    val openCommandAuto =
        Sequential(
            Parallel(
                openIntakeArm,
                openIntakeClaw,
                avoidBasket,
                RunNonBlocking(closeSlides)
            ),
            openExtendo,
            Wait(0.4),
            moveToTransfer
        )

    @JvmStatic val reset = Parallel(
        closeExtendo,
        closeIntakeArm,
    )
    @JvmStatic
    val closeCommand =
        Parallel(
            closeExtension,
            InstantCommand{RobotVariables.extendo = "close"},
            flip,
            closeExtendo,
            moveArmToTransfer,
            closeIntakeArm,
            openDepositClaw,
            closeSlides,
            Sequential(
                IfElse(
                    { transferState == TransferState.TRANSFER },
                    Sequential(
                        RunNonBlocking(
                            Sequential(
                                Wait(0.2),
                                openExtension
                            )
                        ),
                        closeWhenSampleInPlace.raceWith(Wait(0.8)),
                        closeDepositClaw,
                        Wait(0.05),
                        openIntakeClaw,
                        Wait(0.1),
                        enablePID,
                        RunNonBlocking(smartDeposit),
//                        postTransferSequence,
                        Wait(0.6),
                        fullyCloseDepositClaw,
                        WaitUntil{(abs(Mercurial.gamepad2.rightStickY.state) >0.2 ||(LiftVariables.targetPosition>500 && abs(
                            LiftVariables.targetPosition- LiftHardware.getPose()
                        ) < RobotVariables.deltaToReopenAfterSwitch) || (RobotVariables.gameElement == GameElement.SPECIMEN && LiftHardware.getPose() >1000) )},

                    ),
                    Wait(0.0),


                )
            )

        )

    val macro = SuperAdvancing(closeCommand, openCommand)
    @JvmStatic
    val openPushCommand = Parallel(
        openExtendo,
        armUp
    )

    @JvmStatic
    val closeNoTransfer =
        Parallel(
            setPartialOpen,
            doTransfer,
            InstantCommand{ closeCommand.schedule() }
        )
    val partialTransfer = Parallel(
        setPartialOpen,
        doTransfer,
        InstantCommand{ macro.schedule() }
    )
    val partialNoTransfer = Parallel(
        setPartialOpen,
        doNothing,
        InstantCommand{ macro.schedule() }
    )

    val fullOpenTransfer = Parallel(
        setFullOpen,
        doTransfer,
        InstantCommand{ macro.schedule() }
    )

    val fullOpenNoTransfer = Parallel(
        setFullOpen,
        doNothing,
        InstantCommand{ macro.schedule() }
    )
    @JvmStatic
    val wallSeq = Parallel(
        v4bWall,
        openDepositClaw,
        enablePID,
        moveToWall,
        goToWall
    )
    @JvmStatic
    val wallSeqSimple = Parallel(
        openDepositClaw,
        enablePID,
        moveToWall,
        goToWall
    )
    @JvmStatic
    val chamberSeq =
        Sequential(
            closeDepositClaw,
            Wait(0.04),
            enablePID,
            moveToChamber,
            goToHighChamber,
            quickRC()
        )
    @JvmStatic
    val up =
    Sequential(
        RunNonBlocking(
            InstantCommand { targetPosition +=10000}
        ),
//        Wait(0.1)
    )
    @JvmStatic
    val chamberSeqNoDelay =
        Sequential(
            closeDepositClaw,
            enablePID,
            moveToChamber,
            goToHighChamber,
            quickRC()
        )
    val wallToChamber =
        IfElse(
            { abs(armServoLeft.position - wallArmPosition)>0.1 },
            wallSeq,
            chamberSeq
        )
//    @JvmStatic
//    val slamSeq = Sequential(moveToSlam, Wait(0.0), down, Wait(0.15), openDepositClaw)

}