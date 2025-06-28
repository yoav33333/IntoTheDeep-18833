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
//import org.firstinspires.ftc.teamcode.subsystems.robot.RobotCommands.slamSeq
import kotlin.math.round

object DepositClawCommands {
    @JvmStatic
    val closeDepositClaw = Lambda("CloseClaw")
        .setInit {
            setDepositClawPosition(DepositClawVariables.closedClawPosition)
        }
        .setRunStates(Wrapper.OpModeState.ACTIVE, Wrapper.OpModeState.INIT)
    @JvmStatic
    val fullyCloseDepositClaw = Lambda("fullyCloseClaw")
        .setInit {
            setDepositClawPosition(DepositClawVariables.fullyClosedClawPosition)
        }
        .setRunStates(Wrapper.OpModeState.ACTIVE, Wrapper.OpModeState.INIT)
    @JvmStatic
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
            if (round(getDepositClawPosition()) == round(closedClawPosition)) {
                openDepositClaw.schedule()
            } else {
                closeDepositClaw.schedule()
            }
        }
    @JvmStatic
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
    @JvmStatic
    fun quickRC(supplier: () -> Boolean = {true}) = Sequential(WaitUntil(supplier),
        semiCloseClaw, Wait(0.3), closeDepositClaw)

}