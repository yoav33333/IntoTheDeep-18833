package org.firstinspires.ftc.teamcode.util

import dev.frozenmilk.mercurial.commands.Lambda
import java.util.function.BooleanSupplier

object  utilCommands {
    fun waitUntil(supplier: BooleanSupplier) =
        Lambda("Wait until")
            .setFinish { supplier.asBoolean }

}