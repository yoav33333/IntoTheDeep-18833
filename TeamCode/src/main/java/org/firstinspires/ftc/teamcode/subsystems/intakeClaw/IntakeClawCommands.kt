package org.firstinspires.ftc.teamcode.subsystems.intakeClaw

import dev.frozenmilk.dairy.core.wrapper.Wrapper
import dev.frozenmilk.mercurial.commands.Lambda
import dev.frozenmilk.mercurial.commands.groups.Sequential
import dev.frozenmilk.mercurial.commands.util.IfElse
import dev.frozenmilk.mercurial.commands.util.Wait
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.commands.util.InstantCommand
import org.firstinspires.ftc.teamcode.commands.util.RunNonBlocking
import org.firstinspires.ftc.teamcode.subsystems.intakeClaw.IntakeClawHardware.getIntakeClawPosition
import org.firstinspires.ftc.teamcode.subsystems.intakeClaw.IntakeClawHardware.getRotationPosition
import org.firstinspires.ftc.teamcode.subsystems.intakeClaw.IntakeClawHardware.intakeClaw

import org.firstinspires.ftc.teamcode.subsystems.intakeClaw.IntakeClawHardware.setIntakeClawPosition
import org.firstinspires.ftc.teamcode.subsystems.intakeClaw.IntakeClawHardware.setRotationPosition
import org.firstinspires.ftc.teamcode.subsystems.intakeClaw.IntakeClawVariables.closedClawPosition
import org.firstinspires.ftc.teamcode.subsystems.intakeClaw.IntakeClawVariables.openedClawPosition
import org.firstinspires.ftc.teamcode.subsystems.v4b.V4bCommands.moveArmOut
import org.firstinspires.ftc.teamcode.subsystems.v4b.V4bCommands.moveArmOutIntake
import kotlin.math.round


object IntakeClawCommands {

    val openIntakeClaw = Lambda("openClaw")
        .setRunStates(Wrapper.OpModeState.ACTIVE)
        .setInit { setIntakeClawPosition(IntakeClawVariables.openedClawPosition) }
    @JvmStatic
    val closeIntakeClaw = Lambda("closeClaw")
        .setRunStates(Wrapper.OpModeState.ACTIVE)
        .setInit { setIntakeClawPosition(IntakeClawVariables.closedClawPosition) }
    @JvmStatic
    val resetAngleClaw = Lambda("resetAngleClaw")
        .setInit { setRotationPosition { IntakeClawVariables.centeredRotation } }

    val rotateClawL = Lambda("rotate claw l")
        .setRunStates(Wrapper.OpModeState.ACTIVE)
        .setInit {
            setRotationPosition { if (getRotationPosition() <= 1.0) getRotationPosition() + 0.125 else 1.0 }
        }
    val rotateClawR = Lambda("rotate claw r")
        .setRunStates(Wrapper.OpModeState.ACTIVE)
        .setInit {
            setRotationPosition { if (getRotationPosition() >= 0.0) getRotationPosition() - 0.125 else 0.0 }
        }

    val closingClawSeq = Sequential(
        RunNonBlocking( moveArmOutIntake ),
        Wait(0.05),
        closeIntakeClaw,
        Wait(0.1),
        RunNonBlocking(moveArmOut)
    )

    val changeClawIntakePos = Lambda("changeClawPos")
        .setRunStates(Wrapper.OpModeState.ACTIVE)
        .setInit {
            if (round(intakeClaw.position) == round(openedClawPosition)) {
                closingClawSeq.schedule()
            } else {
                openIntakeClaw.schedule()
            }
        }


//    Lambda("changeClawPos")
//        .setRunStates(Wrapper.OpModeState.ACTIVE)
//        .setInit {
//            if (intakeClaw.position == IntakeClawVariables.closedClawPosition) {
//                openIntakeClaw.schedule()
//                return@setInit
//            }
//            else {
//                closingClawSeq.schedule()
//            }
//        }
//        .setFinish{true}


}