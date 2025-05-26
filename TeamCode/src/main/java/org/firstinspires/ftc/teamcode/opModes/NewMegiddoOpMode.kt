package org.firstinspires.ftc.teamcode.opModes

import dev.frozenmilk.mercurial.Mercurial
import org.firstinspires.ftc.teamcode.subsystems.antony.AntonyHardware
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmHardware
import org.firstinspires.ftc.teamcode.subsystems.depositClaw.DepositClawHardware
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveHardware
import org.firstinspires.ftc.teamcode.subsystems.extendo.ExtendoHardware
import org.firstinspires.ftc.teamcode.subsystems.intakeClaw.IntakeClawHardware
import org.firstinspires.ftc.teamcode.subsystems.lift.LiftHardware
import org.firstinspires.ftc.teamcode.subsystems.robot.BulkReads
import org.firstinspires.ftc.teamcode.subsystems.robot.Robot
import org.firstinspires.ftc.teamcode.subsystems.robot.Telemetry
import org.firstinspires.ftc.teamcode.subsystems.v4b.V4bHardware

@Mercurial.Attach
@AntonyHardware.Attach
@BulkReads.Attach
@ArmHardware.Attach
@DepositClawHardware.Attach
@DriveHardware.Attach
@ExtendoHardware.Attach
@IntakeClawHardware.Attach
@LiftHardware.Attach
@Robot.Attach
@Telemetry.Attach
@V4bHardware.Attach

open class NewMegiddoOpMode: CommandOpMode(
    Mercurial,
    AntonyHardware,
    BulkReads,
    ArmHardware,
    DepositClawHardware,
    DriveHardware,
    ExtendoHardware,
    IntakeClawHardware,
    LiftHardware,
    Robot,
    Telemetry,
    V4bHardware
    )