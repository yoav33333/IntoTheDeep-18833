package org.firstinspires.ftc.teamcode.subsystems.arm

import dev.frozenmilk.dairy.core.wrapper.Wrapper
import dev.frozenmilk.mercurial.commands.Lambda
import dev.frozenmilk.mercurial.commands.groups.Parallel
import dev.frozenmilk.mercurial.commands.groups.Sequential
import dev.frozenmilk.mercurial.commands.util.IfElse
import dev.frozenmilk.mercurial.commands.util.Wait
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmHardware.armServoLeft
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmHardware.setArmPosition
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmHardware.setArmPositionCommand
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmHardware.setExtendingArmPosition
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmHardware.setExtensionPosition
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmVariables.armLowBasket
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmVariables.depositArmPosition
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmVariables.depositHighArmPosition
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmVariables.armTarget
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmVariables.avoidBasketExtensionPosition
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmVariables.avoidBasketPosition
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmVariables.chamberArmPosition
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmVariables.chamberExtensionPosition
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmVariables.closeExtensionPosition
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmVariables.depositExtensionPosition
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmVariables.depositHighExtensionPosition
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmVariables.openExtensionPosition
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmVariables.slamArmPosition
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmVariables.transferArmPosition
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmVariables.transferExtensionPosition
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmVariables.wallArmPosition
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmVariables.wallExtensionPosition
import org.firstinspires.ftc.teamcode.subsystems.lift.LiftCommands.enablePID
import org.firstinspires.ftc.teamcode.subsystems.lift.LiftCommands.goToHighChamber
import org.firstinspires.ftc.teamcode.subsystems.lift.LiftCommands.goToWall

object ArmCommands {
    val moveToTransfer = Sequential(
        setExtendingArmPosition({ transferArmPosition},{ transferExtensionPosition}),
    )
    val moveArmToTransfer = setArmPositionCommand { transferArmPosition }
    @JvmStatic val avoidBasket = setExtendingArmPosition({ avoidBasketPosition },
        { avoidBasketExtensionPosition })
    @JvmStatic val moveToDeposit = setExtendingArmPosition({ depositArmPosition }, { depositExtensionPosition })
    val moveToDepositHigh = setExtendingArmPosition(
        { depositHighArmPosition },
        { depositHighExtensionPosition })
    @JvmStatic
    val moveToSlam = setExtendingArmPosition({ slamArmPosition })
    @JvmStatic
    val moveToWall = setExtendingArmPosition({ wallArmPosition }, { wallExtensionPosition})
    @JvmStatic
    val moveToChamber = setExtendingArmPosition({ chamberArmPosition}, { chamberExtensionPosition})
    val moveToLowBasket = setArmPositionCommand{ armLowBasket}
    val smartDeposit = IfElse(
        { armTarget == ArmTarget.HIGH },
        moveToDepositHigh,
        moveToDeposit
    )

    val closeExtension = Lambda("closeExtension")
        .setInit{setExtensionPosition( closeExtensionPosition)}
    @JvmStatic
    val openExtension = Lambda("openExtension")
        .setInit{setExtensionPosition(openExtensionPosition)}

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