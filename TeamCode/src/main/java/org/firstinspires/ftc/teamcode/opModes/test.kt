package org.firstinspires.ftc.teamcode.opModes

//import org.firstinspires.ftc.teamcode.subsystems.driveSubsystem
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
import org.firstinspires.ftc.teamcode.commands.extendoCommand.extendoMacro
import org.firstinspires.ftc.teamcode.subsystems.armClawSubsystem
import org.firstinspires.ftc.teamcode.subsystems.clawSubsystem
import org.firstinspires.ftc.teamcode.subsystems.clawSubsystem.rotateClawL
import org.firstinspires.ftc.teamcode.subsystems.clawSubsystem.rotateClawR
import org.firstinspires.ftc.teamcode.subsystems.deposit
import org.firstinspires.ftc.teamcode.subsystems.deposit.catchPixel
import org.firstinspires.ftc.teamcode.subsystems.deposit.intakeCommand
import org.firstinspires.ftc.teamcode.subsystems.deposit.postIntakeState
import org.firstinspires.ftc.teamcode.subsystems.deposit.release
import org.firstinspires.ftc.teamcode.subsystems.extendoSubsystem
import org.firstinspires.ftc.teamcode.subsystems.followerSubsystem
import org.firstinspires.ftc.teamcode.subsystems.linearSlides
import org.firstinspires.ftc.teamcode.subsystems.linearSlides.getPose
import org.firstinspires.ftc.teamcode.subsystems.linearSlides.goToHighBasket
import org.firstinspires.ftc.teamcode.subsystems.linearSlides.goToHighChamber
import org.firstinspires.ftc.teamcode.subsystems.linearSlides.goToLowBasket
import org.firstinspires.ftc.teamcode.subsystems.linearSlides.goToLowChamber
import org.firstinspires.ftc.teamcode.subsystems.linearSlides.magneticLimit
import org.firstinspires.ftc.teamcode.subsystems.linearSlides.resetHeight
import org.firstinspires.ftc.teamcode.subsystems.linearSlides.target
import kotlin.math.abs


@TeleOp
@Config

class test : MegiddoOpMode() {
    lateinit var telemetryDB: MultipleTelemetry

    //TODO: add auto closing if pixel intaked
    override fun myInit() {
        //operator controls
        Mercurial.gamepad2.options.onTrue(deposit.armIn)
        Mercurial.gamepad2.y.onTrue(clawSubsystem.changeClawPos)
        Mercurial.gamepad2.b.onTrue(deposit.changeClawPos)
        Mercurial.gamepad2.leftStickButton.onTrue(clawSubsystem.resetAngleClaw)
        Mercurial.gamepad2.a.onTrue(
            Sequential(
                intakeCommand,
                Wait(0.5),
                catchPixel,
                Wait(0.3),
                postIntakeState
            )
        )
        Mercurial.gamepad2.leftBumper.onTrue(rotateClawL)
        Mercurial.gamepad2.rightBumper.onTrue(rotateClawR)
        Mercurial.gamepad2.x.onTrue(extendoMacro)
        Mercurial.gamepad2.dpadUp.onTrue(goToHighBasket)
        Mercurial.gamepad2.dpadRight.onTrue(goToHighChamber)
        Mercurial.gamepad2.dpadLeft .onTrue(goToLowBasket)
        Mercurial.gamepad2.dpadDown.onTrue(goToLowChamber)
        BoundBooleanSupplier(EnhancedBooleanSupplier { abs(Mercurial.gamepad2.leftStickY.state) > 0.3 })
            .whileTrue(extendoSubsystem.moveManual)

        BoundBooleanSupplier(EnhancedBooleanSupplier { abs(Mercurial.gamepad2.rightStickY.state) > 0.1 })
            .onTrue(linearSlides.manualControl)
        BoundBooleanSupplier(EnhancedBooleanSupplier { abs(Mercurial.gamepad2.rightStickY.state) < 0.1 })
            .onTrue(linearSlides.runToPosition)
        BoundBooleanSupplier(EnhancedBooleanSupplier { !magneticLimit.state })
            .whileTrue(resetHeight)

        //drive controls
        BoundBooleanSupplier(EnhancedBooleanSupplier { Mercurial.gamepad1.rightTrigger.state >0.2 })
            .onTrue(followerSubsystem.secondGear)
            .onFalse(followerSubsystem.firstGear)
        Mercurial.gamepad1.rightBumper.onTrue(followerSubsystem.secondGear)
            .onFalse(followerSubsystem.firstGear)
        Mercurial.gamepad1.rightStickButton.onTrue(deposit.changeClawPos)

        Mercurial.gamepad1.dpadUp.onTrue( followerSubsystem.angleReset)
        Mercurial.gamepad1.leftBumper.onTrue( followerSubsystem.angleReset)
        Mercurial.gamepad1.leftStickButton.onTrue(
            Sequential(
                deposit.closeH,
                Wait(0.3),
                postIntakeState
            )
        )

        telemetryDB = MultipleTelemetry(
            FeatureRegistrar.activeOpMode.telemetry,
            FtcDashboard.getInstance().telemetry
        )

    }

    override fun myStart() {
        extendoMacro.schedule()
        followerSubsystem.teleopDrive.schedule()
    }
    override fun myLoop() {
        telemetryDB.addData("clawPosDepo", deposit.depoClawServo.position)
        telemetryDB.addData("clawPos", clawSubsystem.clawServo.position)
        telemetryDB.addData("v4b", armClawSubsystem.armClawServo.position)
        telemetryDB.addData("v4b flip", armClawSubsystem.angleClawServo.position)
        telemetryDB.addData("ex r", extendoSubsystem.extendoServoR.position)
        telemetryDB.addData("ex l", extendoSubsystem.extendoServoL.position)
        telemetryDB.addData("offset", linearSlides.offset)
        telemetryDB.addData("sensor", magneticLimit.state)
        telemetryDB.addData("sch", Mercurial.isScheduled(linearSlides.runToPosition))
        telemetryDB.addData("l1", linearSlides.motorLiftNear.power)
        telemetryDB.addData("l2", linearSlides.motorLiftMiddle.power)
        telemetryDB.addData("l3", linearSlides.motorLiftFar.power)
        telemetryDB.addData("rotate", clawSubsystem.clawRotationServo.position)
        telemetryDB.addData("pose", getPose())
        telemetryDB.addData("target", target)
        telemetryDB.addData("error", target - getPose())
        telemetryDB.addData("deposit claw", deposit.depoClawServo.position)
        telemetryDB.update()


    }

}