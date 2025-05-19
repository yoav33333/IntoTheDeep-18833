package org.firstinspires.ftc.teamcode.subsystems.intakeClaw

import dev.frozenmilk.dairy.core.wrapper.Wrapper
import dev.frozenmilk.mercurial.commands.Lambda
import dev.frozenmilk.mercurial.commands.groups.Sequential
import dev.frozenmilk.mercurial.commands.util.Wait
import org.firstinspires.ftc.teamcode.subsystems.deposit.closeH
import org.firstinspires.ftc.teamcode.subsystems.deposit.releaseH
import org.firstinspires.ftc.teamcode.subsystems.intakeClaw.IntakeClawHardware.getIntakeClawPosition
import org.firstinspires.ftc.teamcode.subsystems.intakeClaw.IntakeClawHardware.getRotationPosition

import org.firstinspires.ftc.teamcode.subsystems.intakeClaw.IntakeClawHardware.setIntakeClawPosition
import org.firstinspires.ftc.teamcode.subsystems.intakeClaw.IntakeClawHardware.setRotationPosition
import org.firstinspires.ftc.teamcode.subsystems.v4b.V4bCommands.moveArmOut
import org.firstinspires.ftc.teamcode.subsystems.v4b.V4bCommands.moveArmOutIntake


object IntakeClawCommands {

    val openIntakeClaw = Lambda("openClaw")
        .setRunStates(Wrapper.OpModeState.ACTIVE)
        .setInit { setIntakeClawPosition(IntakeClawVariables.openedClawPosition) }

    val closeIntakeClaw = Lambda("closeClaw")
        .setRunStates(Wrapper.OpModeState.ACTIVE)
        .setInit { setIntakeClawPosition(IntakeClawVariables.openedClawPosition) }

    val resetAngleClaw = Lambda("resetAngleClaw")
        .setInit { setRotationPosition(IntakeClawVariables.centeredRotation) }

    val rotateClawL = Lambda("rotate claw l")
        .setRunStates(Wrapper.OpModeState.ACTIVE)
        .setInit {
            setRotationPosition(if (getRotationPosition() <= 1.0) getRotationPosition() + 0.125 else 1.0)
        }
    val rotateClawR = Lambda("rotate claw r")
        .setRunStates(Wrapper.OpModeState.ACTIVE)
        .setInit {
            setRotationPosition(if (getRotationPosition() >= 0.0) getRotationPosition() - 0.125 else 0.0)
        }

    val closingClawSeq = Sequential(
        moveArmOutIntake,
        Wait(0.05),
        closeIntakeClaw,
        Wait(0.1),
        moveArmOut
    )

    val changeClawIntakePos = Lambda("changeClawPos")
        .setRunStates(Wrapper.OpModeState.ACTIVE)
        .setEnd {
            if (getIntakeClawPosition() == IntakeClawVariables.closedClawPosition) {
                openIntakeClaw.schedule()
            } else {
                closingClawSeq.schedule()
            }
        }


}