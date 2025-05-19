package org.firstinspires.ftc.teamcode.subsystems.arm

import dev.frozenmilk.dairy.core.wrapper.Wrapper
import dev.frozenmilk.mercurial.commands.Lambda
import dev.frozenmilk.mercurial.commands.util.IfElse
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmHardware.setArmPosition
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmHardware.setArmPositionCommand
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmVariables.DepositArmPosition
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmVariables.DepositHighArmPosition
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmVariables.SlamArmPosition
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmVariables.armTarget
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmVariables.avoidBasketPosition
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmVariables.transferArmPosition

object ArmCommands {
    val moveToTransfer = setArmPositionCommand(transferArmPosition)
    val moveArmToHighDeposit = setArmPositionCommand(DepositHighArmPosition)
    val avoidBasket = setArmPositionCommand(avoidBasketPosition)
    val moveToDeposit = setArmPositionCommand(DepositArmPosition)
    val moveToDepositHigh = setArmPositionCommand(DepositArmPosition)
    val moveToSlam = setArmPositionCommand(SlamArmPosition)

    val smartDeposit = IfElse(
        { armTarget == ArmTarget.HIGH },
        moveToDepositHigh,
        moveToDeposit
    )

    fun setArmTarget(target: ArmTarget) =
        Lambda("Set Arm Target: {}".format(target.toString()))
            .setInit {
                armTarget = target
            }
            .setRunStates(Wrapper.OpModeState.ACTIVE, Wrapper.OpModeState.INIT)

    val toggleArmTarget = IfElse(
        { armTarget == ArmTarget.HIGH },
        setArmTarget(ArmTarget.LOW),
        setArmTarget(ArmTarget.HIGH)
    ).then(smartDeposit)

}