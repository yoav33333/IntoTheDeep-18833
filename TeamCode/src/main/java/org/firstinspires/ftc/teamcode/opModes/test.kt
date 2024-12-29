package org.firstinspires.ftc.teamcode.opModes

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.frozenmilk.dairy.core.FeatureRegistrar
import dev.frozenmilk.dairy.core.util.supplier.logical.EnhancedBooleanSupplier
import dev.frozenmilk.mercurial.Mercurial
import dev.frozenmilk.mercurial.bindings.BoundBooleanSupplier
import dev.frozenmilk.mercurial.commands.groups.Sequential
import dev.frozenmilk.mercurial.commands.util.Wait
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.commands.extendoCommand
import org.firstinspires.ftc.teamcode.commands.extendoCommand.extendoMacro
import org.firstinspires.ftc.teamcode.subsystems.BulkReads
import org.firstinspires.ftc.teamcode.subsystems.armClawSubsystem
import org.firstinspires.ftc.teamcode.subsystems.clawSubsystem

import org.firstinspires.ftc.teamcode.subsystems.clawSubsystem.rotateClawL
import org.firstinspires.ftc.teamcode.subsystems.clawSubsystem.rotateClawR
import org.firstinspires.ftc.teamcode.subsystems.deposit
import org.firstinspires.ftc.teamcode.subsystems.deposit.catchPixel
import org.firstinspires.ftc.teamcode.subsystems.deposit.depoArmServo
import org.firstinspires.ftc.teamcode.subsystems.deposit.intakeCommand
import org.firstinspires.ftc.teamcode.subsystems.deposit.release
import org.firstinspires.ftc.teamcode.subsystems.deposit.transferCommand
import org.firstinspires.ftc.teamcode.subsystems.driveSubsystem
import org.firstinspires.ftc.teamcode.subsystems.extendoSubsystem
import org.firstinspires.ftc.teamcode.subsystems.linearSlides
import org.firstinspires.ftc.teamcode.subsystems.linearSlides.getPose
import org.firstinspires.ftc.teamcode.subsystems.linearSlides.goToHighBasket
import org.firstinspires.ftc.teamcode.subsystems.linearSlides.goToHighChamber
import org.firstinspires.ftc.teamcode.subsystems.linearSlides.goToLowBasket
import org.firstinspires.ftc.teamcode.subsystems.linearSlides.goToLowChamber
import org.firstinspires.ftc.teamcode.subsystems.linearSlides.magneticLimit
import org.firstinspires.ftc.teamcode.subsystems.linearSlides.resetHeight
import org.firstinspires.ftc.teamcode.subsystems.linearSlides.runToPose
import org.firstinspires.ftc.teamcode.subsystems.linearSlides.target
import kotlin.math.abs

@BulkReads.Attach
@Mercurial.Attach
@clawSubsystem.Attach
@driveSubsystem.Attach
@extendoCommand.Attach
@linearSlides.Attach
@armClawSubsystem.Attach
@extendoSubsystem.Attach
@deposit.Attach
@TeleOp
@Config
class test : CommandOpMode(BulkReads, Mercurial, clawSubsystem, driveSubsystem, extendoCommand, linearSlides, armClawSubsystem, extendoSubsystem, deposit) {
    lateinit var telemetryDB : MultipleTelemetry
    override fun myInit() {
        //operator controls
        Mercurial.gamepad2.y.onTrue(clawSubsystem.changeClawPos)
        Mercurial.gamepad2.b.onTrue(deposit.changeClawPos)
        Mercurial.gamepad2.leftStickButton.onTrue(clawSubsystem.resetAngleClaw)
        Mercurial.gamepad2.a.onTrue(Sequential(intakeCommand, Wait(0.5).then(catchPixel)))
        Mercurial.gamepad2.leftBumper.onTrue(rotateClawL)
        Mercurial.gamepad2.rightBumper.onTrue(rotateClawR)
        Mercurial.gamepad2.x.onTrue(extendoMacro)
        Mercurial.gamepad2.dpadUp.onTrue(goToHighBasket)
        Mercurial.gamepad2.dpadLeft.onTrue(goToHighChamber)
        Mercurial.gamepad2.dpadRight.onTrue(goToLowBasket)
        Mercurial.gamepad2.dpadDown.onTrue(goToLowChamber)
        BoundBooleanSupplier(EnhancedBooleanSupplier{ abs(Mercurial.gamepad2.leftStickY.state)>0.3})
            .whileTrue(extendoSubsystem.moveManual)

        BoundBooleanSupplier(EnhancedBooleanSupplier{abs(Mercurial.gamepad2.rightStickY.state)>0.1})
            .whileTrue(linearSlides.manualControl)
        BoundBooleanSupplier(EnhancedBooleanSupplier{abs(Mercurial.gamepad2.rightStickY.state)<0.1})
            .whileTrue(linearSlides.runToPosition)
        BoundBooleanSupplier(EnhancedBooleanSupplier { !magneticLimit.state })
            .onTrue(resetHeight)

//        BoundBooleanSupplier(EnhancedBooleanSupplier{clawSubsystem.colorDistSensor.getDistance(DistanceUnit.MM)<35.0 && clawSubsystem.check})
//            .onTrue(Wait(0.1).then(clawSubsystem.closeClaw))


//        Mercurial.gamepad2.a.onTrue(linearSlides.closeSlides)
        //drive controls
        Mercurial.gamepad1.rightStickButton.onTrue(release)
        Mercurial.gamepad1.leftStickButton.onTrue(release)
        Mercurial.gamepad1.b.onTrue(driveSubsystem.midSlowGear)
        Mercurial.gamepad1.x.onTrue(driveSubsystem.midFastGear)
        Mercurial.gamepad1.a.onTrue(driveSubsystem.fastGear)
        Mercurial.gamepad1.y.onTrue(driveSubsystem.slowGear)

        telemetryDB = MultipleTelemetry(FeatureRegistrar.activeOpMode.telemetry, FtcDashboard.getInstance().telemetry)

    }
    override fun myLoop() {
        telemetry.addData("clawPosDepo", deposit.depoClawServo.position)
        telemetry.addData("clawPos", clawSubsystem.clawServo.position)
        telemetry.addData("v4b", armClawSubsystem.armClawServo.position)
        telemetry.addData("v4b flip", armClawSubsystem.angleClawServo.position)
        telemetry.addData("ex r", extendoSubsystem.extendoServoR.position)
        telemetry.addData("ex l", extendoSubsystem.extendoServoL.position)
        telemetry.addData("offset", linearSlides.offset)
        telemetry.addData("sensor", linearSlides.magneticLimit.state)
        telemetry.addData("sch", Mercurial.isScheduled(linearSlides.runToPosition))

        telemetry.update()

//        runToPose(target.toDouble())
        telemetryDB.addData("pose", getPose())
        telemetryDB.addData("target", target)
        telemetryDB.addData("error", target - getPose())
        telemetryDB.addData("red", clawSubsystem.colorDistSensor.red())
        telemetryDB.addData("blue", clawSubsystem.colorDistSensor.blue())
        telemetryDB.addData("green", clawSubsystem.colorDistSensor.green())
        telemetryDB.update()

    }

}