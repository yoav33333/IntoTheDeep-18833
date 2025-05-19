package org.firstinspires.ftc.teamcode.opModes

import dev.frozenmilk.dairy.core.util.supplier.logical.EnhancedBooleanSupplier
import dev.frozenmilk.mercurial.Mercurial
import dev.frozenmilk.mercurial.bindings.BoundBooleanSupplier
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmCommands.avoidBasket
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmCommands.moveToTransfer
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmCommands.toggleArmTarget
import org.firstinspires.ftc.teamcode.subsystems.depositClaw.DepositClawCommands.quickRC
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveCommands.resetHeading
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
import org.firstinspires.ftc.teamcode.subsystems.lift.LiftCommands.manualControl
import org.firstinspires.ftc.teamcode.subsystems.lift.LiftCommands.switchBasket
import org.firstinspires.ftc.teamcode.subsystems.robot.RobotCommands.doNothing

import kotlin.math.abs

class TerribleTeleop: NewMegiddoOpMode() {
    override fun myInit() {
        /*operator controls*/
        Mercurial.gamepad2.options.onTrue(moveToTransfer)
        Mercurial.gamepad2.y.whileTrue(changeClawIntakePos)
        Mercurial.gamepad2.b.onTrue(doNothing)
        Mercurial.gamepad2.leftStickButton.onTrue(resetAngleClaw)
        Mercurial.gamepad2.a.onTrue(setPartialOpen.then(openExtendo))
        Mercurial.gamepad2.leftBumper.onTrue(rotateClawL)
        Mercurial.gamepad2.rightBumper.onTrue(rotateClawR)
        Mercurial.gamepad2.x.onTrue(setFullOpen.then(openExtendo))
        Mercurial.gamepad2.dpadUp.onTrue(goToBasket)
        Mercurial.gamepad2.dpadDown.onTrue(goToHighChamberUp)
        Mercurial.gamepad2.leftStickButton.onTrue(closeSlides.with(avoidBasket))
        Mercurial.gamepad2.share.onTrue(quickRC())
        BoundBooleanSupplier(EnhancedBooleanSupplier { abs(Mercurial.gamepad2.leftStickY.state) > 0.3 })
            .whileTrue(manualMove)
        BoundBooleanSupplier(EnhancedBooleanSupplier { abs(Mercurial.gamepad2.rightStickY.state) > 0.1 })
            .onTrue(manualControl)



        /*drive controls*/
        Mercurial.gamepad1.rightBumper.onTrue(switchBasket)
        Mercurial.gamepad1.rightStickButton.onTrue(changeClawIntakePos)
        Mercurial.gamepad1.dpadUp.onTrue( resetHeading)
        Mercurial.gamepad1.leftBumper.onTrue( resetHeading)
        Mercurial.gamepad1.leftStickButton.onTrue(toggleArmTarget)


    }
}