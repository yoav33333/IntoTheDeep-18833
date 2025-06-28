package org.firstinspires.ftc.teamcode.opModes

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.frozenmilk.dairy.core.util.supplier.logical.EnhancedBooleanSupplier
import dev.frozenmilk.mercurial.Mercurial
import dev.frozenmilk.mercurial.bindings.BoundBooleanSupplier
import dev.frozenmilk.mercurial.commands.groups.Sequential
import dev.frozenmilk.mercurial.commands.util.IfElse
import dev.frozenmilk.mercurial.commands.util.Wait
import org.firstinspires.ftc.teamcode.commands.util.WaitUntil
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmCommands.avoidBasket
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmCommands.moveToDeposit
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmCommands.moveToLowBasket
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmCommands.moveToTransfer
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmCommands.toggleArmTarget
import org.firstinspires.ftc.teamcode.subsystems.depositClaw.DepositClawCommands.changeClawPos
import org.firstinspires.ftc.teamcode.subsystems.depositClaw.DepositClawCommands.quickRC
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveCommands.resetHeading
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveCommands.runLowGear
import org.firstinspires.ftc.teamcode.subsystems.extendo.ExtendoCommands.manualMove
import org.firstinspires.ftc.teamcode.subsystems.extendo.ExtendoCommands.openExtendo
import org.firstinspires.ftc.teamcode.subsystems.extendo.ExtendoCommands.setFullOpen
import org.firstinspires.ftc.teamcode.subsystems.extendo.ExtendoCommands.setPartialOpen
import org.firstinspires.ftc.teamcode.subsystems.intakeClaw.IntakeClawCommands.changeClawIntakePos
import org.firstinspires.ftc.teamcode.subsystems.intakeClaw.IntakeClawCommands.resetAngleClaw
import org.firstinspires.ftc.teamcode.subsystems.intakeClaw.IntakeClawCommands.rotateClawL
import org.firstinspires.ftc.teamcode.subsystems.intakeClaw.IntakeClawCommands.rotateClawR
import org.firstinspires.ftc.teamcode.subsystems.lift.LiftCommands.closeSlides
import org.firstinspires.ftc.teamcode.subsystems.lift.LiftCommands.goToBasket
import org.firstinspires.ftc.teamcode.subsystems.lift.LiftCommands.goToHighChamberUp
import org.firstinspires.ftc.teamcode.subsystems.lift.LiftCommands.goToLowBasket
import org.firstinspires.ftc.teamcode.subsystems.lift.LiftCommands.manualControl
import org.firstinspires.ftc.teamcode.subsystems.lift.LiftCommands.switchBasket
import org.firstinspires.ftc.teamcode.subsystems.lift.LiftState
import org.firstinspires.ftc.teamcode.subsystems.lift.LiftVariables.liftState
import org.firstinspires.ftc.teamcode.subsystems.robot.RobotCommands.doNothing
import org.firstinspires.ftc.teamcode.subsystems.robot.RobotCommands.doTransfer
import org.firstinspires.ftc.teamcode.subsystems.robot.RobotCommands.fullOpenNoTransfer
import org.firstinspires.ftc.teamcode.subsystems.robot.RobotCommands.fullOpenTransfer
import org.firstinspires.ftc.teamcode.subsystems.robot.RobotCommands.macro
import org.firstinspires.ftc.teamcode.subsystems.robot.RobotCommands.partialNoTransfer
import org.firstinspires.ftc.teamcode.subsystems.robot.RobotCommands.partialTransfer
import org.firstinspires.ftc.teamcode.subsystems.robot.RobotCommands.wallToChamber

import kotlin.math.abs
@TeleOp
class TerribleTeleop: NewMegiddoOpMode() {
    override fun myInit() {
        /*operator controls*/
        Mercurial.gamepad2.options.onTrue(moveToTransfer)
        Mercurial.gamepad2.y.onTrue(changeClawIntakePos)
        Mercurial.gamepad2.a.onTrue(partialNoTransfer)
        Mercurial.gamepad2.leftStickButton.onTrue(resetAngleClaw)
        Mercurial.gamepad2.b.onTrue(wallToChamber)
        Mercurial.gamepad2.leftBumper.onTrue(rotateClawL)
        Mercurial.gamepad2.rightBumper.onTrue(rotateClawR)
        Mercurial.gamepad2.x.onTrue(fullOpenTransfer)
        Mercurial.gamepad2.dpadUp.onTrue(
            Sequential(
                IfElse(
                    {liftState != LiftState.AUTO}
                    ,Sequential(
                        WaitUntil{liftState == LiftState.AUTO },
                    ),
                    Wait(0.0)),
                moveToDeposit,
                goToBasket
            )
        )
        Mercurial.gamepad2.dpadDown.onTrue(
            Sequential(
                IfElse(
                    {liftState != LiftState.AUTO}
                    ,Sequential(
                        WaitUntil{liftState == LiftState.AUTO },
                        Wait(0.5)),
                    Wait(0.0)),
                moveToLowBasket,
                goToLowBasket)
        )
        Mercurial.gamepad2.leftStickButton.onTrue(closeSlides.with(avoidBasket))
        Mercurial.gamepad2.share.onTrue(quickRC())
        BoundBooleanSupplier(EnhancedBooleanSupplier { abs(Mercurial.gamepad2.leftStickY.state) > 0.3 })
            .whileTrue(manualMove)
        BoundBooleanSupplier(EnhancedBooleanSupplier { abs(Mercurial.gamepad2.rightStickY.state) > 0.1 })
            .onTrue(manualControl)

        /*drive controls*/
        BoundBooleanSupplier(EnhancedBooleanSupplier{Mercurial.gamepad1.rightTrigger.state > 0.2})
            .whileTrue(runLowGear)
        Mercurial.gamepad1.rightBumper.onTrue(switchBasket)
        Mercurial.gamepad1.rightStickButton.onTrue(changeClawPos)
        Mercurial.gamepad1.dpadUp.onTrue( resetHeading)
        Mercurial.gamepad1.leftBumper.onTrue( resetHeading)
        Mercurial.gamepad1.leftStickButton.onTrue(toggleArmTarget)
    }
}