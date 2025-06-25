package org.firstinspires.ftc.teamcode.subsystems.v4b

import dev.frozenmilk.mercurial.commands.Lambda
import dev.frozenmilk.mercurial.commands.groups.Parallel
import dev.frozenmilk.mercurial.commands.groups.Sequential
import dev.frozenmilk.mercurial.commands.util.Wait

import org.firstinspires.ftc.teamcode.subsystems.v4b.V4bHardware.setArmPosition


import org.firstinspires.ftc.teamcode.subsystems.v4b.V4bHardware.setPitchPosition
import org.firstinspires.ftc.teamcode.subsystems.v4b.V4bVariables.armInPosition
import org.firstinspires.ftc.teamcode.subsystems.v4b.V4bVariables.armOutPosition
import org.firstinspires.ftc.teamcode.subsystems.v4b.V4bVariables.armPushPosition
import org.firstinspires.ftc.teamcode.subsystems.v4b.V4bVariables.armUpPosition
import org.firstinspires.ftc.teamcode.subsystems.v4b.V4bVariables.armWallPosition
import org.firstinspires.ftc.teamcode.subsystems.v4b.V4bVariables.intakePosition
import org.firstinspires.ftc.teamcode.subsystems.v4b.V4bVariables.pitchPushPosition
import org.firstinspires.ftc.teamcode.subsystems.v4b.V4bVariables.postTransferPosition
import org.firstinspires.ftc.teamcode.subsystems.v4b.V4bVariables.transferPosition
import org.firstinspires.ftc.teamcode.subsystems.v4b.V4bVariables.wallPosition

object V4bCommands {
    val pitchTransfer = Lambda("angleTransfer")
        .setInit {
            setPitchPosition { transferPosition }
        }
    val v4bWall =  Lambda("pitchWall")
        .setInit{
            setPitchPosition{ wallPosition}
        setArmPosition{armWallPosition}}
    val pitchPostTransfer = Lambda("anglePostTransfer")
        .setInit {
            setPitchPosition { postTransferPosition }
        }
    val pitchIntake = Lambda("angleIntake")
        .setInit {
            setPitchPosition { intakePosition }
        }
    val moveArmOut = Lambda("moveArmOut")
        .setInit {
            setArmPosition { armOutPosition }
        }
    val moveArmOutIntake = Lambda("moveArmOutIntake")
        .setInit {
            setArmPosition { armOutPosition - 0.02 }
        }
    val moveArmIn = Lambda("moveArmIn")
        .setInit {
            setArmPosition { armInPosition }
        }
    @JvmStatic
    val extendoPush = Lambda("eps")
        .setInit{
            setArmPosition { armPushPosition }
            setPitchPosition { pitchPushPosition }
        }
    val openIntakeArm = Parallel(
        moveArmOut,
        pitchIntake
    )
    val closeIntakeArm = Sequential(
        moveArmIn,
        pitchTransfer
    )

    val postTransferSequence = Sequential(
        pitchPostTransfer,
        Wait(0.1),
        pitchTransfer
    )
    @JvmStatic
    val armUp = Lambda("au")
        .setInit{ setArmPosition { armUpPosition } }
}