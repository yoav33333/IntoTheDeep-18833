package org.firstinspires.ftc.teamcode.commands

import dev.frozenmilk.mercurial.commands.groups.Advancing
import dev.frozenmilk.mercurial.commands.groups.Parallel
import dev.frozenmilk.mercurial.commands.groups.Sequential
import dev.frozenmilk.mercurial.commands.util.Wait
import dev.frozenmilk.util.cell.RefCell
import org.firstinspires.ftc.teamcode.subsystems.armClawSubsystem
import org.firstinspires.ftc.teamcode.subsystems.clawSubsystem
import org.firstinspires.ftc.teamcode.subsystems.deposit.transferCommand
import org.firstinspires.ftc.teamcode.subsystems.extendoSubsystem
//import org.firstinspires.ftc.teamcode.subsystems.extendoSubsystem
//import org.firstinspires.ftc.teamcode.subsystems.extendoSubsystem
import java.lang.annotation.Inherited

object extendoCommand {


    @Target(AnnotationTarget.CLASS)
    @Retention(AnnotationRetention.RUNTIME)
    @MustBeDocumented
    @Inherited
    annotation class Attach

    val extendoOpenCommand = Parallel(
        clawSubsystem.openClaw,
        clawSubsystem.resetAngleClaw,
        armClawSubsystem.openClawArm,
        extendoSubsystem.openExtendo
    )

    val extendoCloseCommand = Sequential(Parallel(
        clawSubsystem.closeClaw,
        clawSubsystem.resetAngleClaw,
        extendoSubsystem.closeExtendo
    ),
        Wait(0.2),
        armClawSubsystem.closeClawArm,
        transferCommand
    )

    val extendoMacro = Advancing(
        extendoCloseCommand,
        extendoOpenCommand
    )

}