package org.firstinspires.ftc.teamcode.subsystems.depositClaw

import dev.frozenmilk.dairy.core.wrapper.Wrapper
import dev.frozenmilk.dairy.pasteurized.Pasteurized
import dev.frozenmilk.mercurial.commands.Lambda
import dev.frozenmilk.mercurial.commands.groups.Sequential
import dev.frozenmilk.mercurial.commands.util.Wait
import org.firstinspires.ftc.teamcode.commands.util.WaitUntil

import org.firstinspires.ftc.teamcode.subsystems.depositClaw.DepositClawHardware.getDepositClawPosition
import org.firstinspires.ftc.teamcode.subsystems.depositClaw.DepositClawHardware.isInRange
import org.firstinspires.ftc.teamcode.subsystems.depositClaw.DepositClawHardware.setDepositClawPosition
import org.firstinspires.ftc.teamcode.subsystems.depositClaw.DepositClawVariables.closedClawPosition
import org.firstinspires.ftc.teamcode.subsystems.robot.GameElement
import org.firstinspires.ftc.teamcode.subsystems.robot.RobotCommands.slamSeq
import org.firstinspires.ftc.teamcode.subsystems.robot.RobotVariables.gameElement

object DepositClawCommands {
    val closeDepositClaw = Lambda("CloseClaw")
        .setInit {
            setDepositClawPosition(DepositClawVariables.closedClawPosition)
        }
        .setRunStates(Wrapper.OpModeState.ACTIVE, Wrapper.OpModeState.INIT)

    val openDepositClaw = Lambda("OpenClaw")
        .setInit {
            setDepositClawPosition(DepositClawVariables.openedClawPosition)
        }
        .setRunStates(Wrapper.OpModeState.ACTIVE, Wrapper.OpModeState.INIT)

    val semiCloseClaw = Lambda("SemiCloseClaw")
        .setInit {
            setDepositClawPosition(DepositClawVariables.semiClosedClawPosition)
        }
        .setRunStates(Wrapper.OpModeState.ACTIVE, Wrapper.OpModeState.INIT)

    /**
     * Checks if a sample is within range and schedules the claw to close if so.
     *
     * @return `true` if the sample is detected in range and the close command is scheduled; `false` otherwise.
     */
    fun checkIfSampleInPlace(): Boolean {
        if (Pasteurized.gamepad2.a.onTrue){
//            intakeSeq.cancel()
//            intakeSeq.schedule()
            //TODO: add a new intake sequence
        }

        if (isInRange()) {
            closeDepositClaw.schedule()
            return true
        }
        return false
    }

    val changeClawPos = Lambda("changeClawPos")
        .setRunStates(Wrapper.OpModeState.ACTIVE)
        .setInit {
            if (getDepositClawPosition() == closedClawPosition) {
                if (gameElement == GameElement.SPECIMEN) slamSeq.schedule()
                else openDepositClaw.schedule()
            } else {
                closeDepositClaw.schedule()
            }
        }

    val closeWhenSampleInPlace = Lambda("closeWhenSampleInPlace")
        .setRunStates(Wrapper.OpModeState.ACTIVE)
        .setInit {
            openDepositClaw.schedule()
        }
        .setExecute {
        }
        .setFinish {
            checkIfSampleInPlace()
        }
    /**
         * Creates a sequential command that waits for a condition, then performs a semi-close action on the deposit claw twice with a delay.
         *
         * @param supplier A function that returns true when the sequence should proceed. Defaults to always true.
         * @return A sequential command that waits until the condition is met, executes the semi-close claw command, waits 0.3 seconds, and repeats the semi-close action.
         */
        @JvmStatic
    fun quickRC(supplier: () -> Boolean = {true}) = Sequential(WaitUntil(supplier),
        semiCloseClaw, Wait(0.3), semiCloseClaw)

}