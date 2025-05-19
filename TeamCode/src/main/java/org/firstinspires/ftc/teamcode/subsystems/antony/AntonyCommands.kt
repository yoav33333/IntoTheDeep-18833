package org.firstinspires.ftc.teamcode.subsystems.antony

import dev.frozenmilk.dairy.core.wrapper.Wrapper
import dev.frozenmilk.mercurial.commands.Lambda
import org.firstinspires.ftc.teamcode.subsystems.antony.AntonyHardware.setPattern

object AntonyCommands {

    val setDefault = Lambda("setDefault")
        .setInit { setPattern(AntonyVariables.default) }
        .setRunStates(Wrapper.OpModeState.ACTIVE)
        .setFinish { false }
        .setRequirements(AntonyHardware)

    val setEndGame = Lambda("setEndGame")
        .setInit { setPattern(AntonyVariables.endGame) }
        .setRunStates(Wrapper.OpModeState.ACTIVE)
        .setFinish { false }
        .setRequirements(AntonyHardware)

    val setLowBattery = Lambda("setLowBattery")
        .setInit { setPattern(AntonyVariables.lowBattery) }
        .setRunStates(Wrapper.OpModeState.ACTIVE)
        .setFinish { false }
        .setRequirements(AntonyHardware)
}