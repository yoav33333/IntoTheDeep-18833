package org.firstinspires.ftc.teamcode.subsystems.extendo

import dev.frozenmilk.dairy.core.wrapper.Wrapper
import dev.frozenmilk.mercurial.Mercurial
import dev.frozenmilk.mercurial.commands.Lambda
import org.firstinspires.ftc.teamcode.subsystems.extendo.ExtendoHardware.extendoServoL
import org.firstinspires.ftc.teamcode.subsystems.extendo.ExtendoHardware.extendoServoR
import org.firstinspires.ftc.teamcode.subsystems.extendo.ExtendoHardware.setPosition
import org.firstinspires.ftc.teamcode.subsystems.extendo.ExtendoVariables.extendoSpeed
import org.firstinspires.ftc.teamcode.subsystems.extendo.ExtendoVariables.extendoState


object ExtendoCommands {
    val manualMove = Lambda("moveManual")
        .setRunStates(Wrapper.OpModeState.ACTIVE)
        .setExecute {
            if (extendoServoL.position - Mercurial.gamepad2.leftStickY.state / 50.0 > 0.3) {
                extendoServoL.position -= Mercurial.gamepad2.leftStickY.state * extendoSpeed
                extendoServoR.position -= Mercurial.gamepad2.leftStickY.state * extendoSpeed
            }
        }
        .setFinish { false }

    val openExtendo = Lambda("openExtendo")
        .setInit {setPosition(ExtendoVariables.openPosition)}

    val closeExtendo = Lambda("closeExtendo")
        .setInit {setPosition(ExtendoVariables.closePosition)}

    val partialOpenExtendo = Lambda("partialOpenExtendo")
        .setInit {setPosition(ExtendoVariables.partialOpeningPosition)}

    val setPartialOpen = Lambda("setPartialOpen")
        .setInit {extendoState = ExtendoState.PARTIAL_OPEN}
    val setFullOpen = Lambda("setFullOpen")
        .setInit {extendoState = ExtendoState.FULL_OPEN}

}