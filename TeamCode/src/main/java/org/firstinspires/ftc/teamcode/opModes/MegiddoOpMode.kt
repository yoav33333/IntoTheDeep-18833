package org.firstinspires.ftc.teamcode.opModes

//import org.firstinspires.ftc.teamcode.subsystems.driveSubsystem
import dev.frozenmilk.mercurial.Mercurial
import org.firstinspires.ftc.teamcode.commands.extendoCommand
import org.firstinspires.ftc.teamcode.subsystems.BulkReads
import org.firstinspires.ftc.teamcode.subsystems.antonySubsystem
import org.firstinspires.ftc.teamcode.subsystems.armClawSubsystem
import org.firstinspires.ftc.teamcode.subsystems.clawSubsystem
import org.firstinspires.ftc.teamcode.subsystems.deposit
import org.firstinspires.ftc.teamcode.subsystems.extendoSubsystem
import org.firstinspires.ftc.teamcode.subsystems.followerSubsystem
import org.firstinspires.ftc.teamcode.subsystems.linearSlides

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
open class MegiddoOpMode : CommandOpMode(
    BulkReads,
    Mercurial,
    clawSubsystem,
    extendoCommand,
    linearSlides,
    armClawSubsystem,
    extendoSubsystem,
    deposit,
    antonySubsystem,
    followerSubsystem
) {

}