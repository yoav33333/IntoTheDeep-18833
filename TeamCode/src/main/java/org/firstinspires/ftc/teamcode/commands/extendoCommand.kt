package org.firstinspires.ftc.teamcode.commands

import dev.frozenmilk.dairy.core.FeatureRegistrar
import dev.frozenmilk.dairy.core.dependency.Dependency
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation
import dev.frozenmilk.dairy.core.wrapper.Wrapper
import dev.frozenmilk.mercurial.Mercurial
import dev.frozenmilk.mercurial.commands.Lambda
import dev.frozenmilk.mercurial.commands.groups.Parallel
import dev.frozenmilk.mercurial.commands.groups.Sequential
import dev.frozenmilk.mercurial.commands.util.StateMachine
import dev.frozenmilk.mercurial.commands.util.Wait
import dev.frozenmilk.mercurial.subsystems.Subsystem
import dev.frozenmilk.util.cell.RefCell
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.subsystems.armClawSubsystem
import org.firstinspires.ftc.teamcode.subsystems.clawSubsystem
import org.firstinspires.ftc.teamcode.subsystems.deposit.transferCommand
import org.firstinspires.ftc.teamcode.subsystems.extendoSubsystem
//import org.firstinspires.ftc.teamcode.subsystems.extendoSubsystem
//import org.firstinspires.ftc.teamcode.subsystems.extendoSubsystem
import java.lang.annotation.Inherited

object extendoCommand {
    enum class extendoState{
        OPEN,
        CLOSE;
    }
    val currentExtendoState: RefCell<extendoState> = RefCell(extendoState.CLOSE)


    @Target(AnnotationTarget.CLASS)
    @Retention(AnnotationRetention.RUNTIME)
    @MustBeDocumented
    @Inherited
    annotation class Attach
    fun extendoCloseCommand(){

        clawSubsystem.clawRotationServo.position = 0.5
        clawSubsystem.closeClaw()
        armClawSubsystem.closeClawArm()
//        extendoSubsystem.closeExtendoF()

    }
    fun extendoOpenCommand(){
        clawSubsystem.clawRotationServo.position = 0.5
        clawSubsystem.openClaw()
        armClawSubsystem.openClawArm()
//        extendoSubsystem.openExtendoF()
    }
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

}