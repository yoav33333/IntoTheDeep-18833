package org.firstinspires.ftc.teamcode.util

import dev.frozenmilk.mercurial.commands.Command
import dev.frozenmilk.mercurial.commands.Lambda
import java.util.function.BooleanSupplier

object  utilCommands {
    fun waitUntil(supplier: BooleanSupplier) =
        Lambda("Wait until")
            .setFinish { supplier.asBoolean }

    fun runNonBlocking(vararg command: Command) =
        Lambda("Run non-blocking")
            .setInit{ command.forEach { it.schedule() } }
    fun instantCommand(vararg command: Runnable)=
        Lambda("instant command")
            .setInit{command.forEach { it.run() }}

}