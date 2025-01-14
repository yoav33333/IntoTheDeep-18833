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
import org.firstinspires.ftc.teamcode.subsystems.antonySubsystem
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
//import org.firstinspires.ftc.teamcode.subsystems.driveSubsystem
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
import org.firstinspires.ftc.teamcode.subsystems.linearSlides.runToPose
import org.firstinspires.ftc.teamcode.subsystems.linearSlides.target
import kotlin.math.abs

@BulkReads.Attach
@Mercurial.Attach
@clawSubsystem.Attach
//@driveSubsystem.Attach
@extendoCommand.Attach
@antonySubsystem.Attach
@linearSlides.Attach
@armClawSubsystem.Attach
@extendoSubsystem.Attach
@deposit.Attach
@followerSubsystem.Attach
open class MegiddoOpMode : CommandOpMode(BulkReads, Mercurial, clawSubsystem, extendoCommand, linearSlides, armClawSubsystem, extendoSubsystem, deposit, antonySubsystem, followerSubsystem) {

}