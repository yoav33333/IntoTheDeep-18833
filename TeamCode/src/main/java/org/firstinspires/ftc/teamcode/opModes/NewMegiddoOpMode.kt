package org.firstinspires.ftc.teamcode.opModes

import org.firstinspires.ftc.teamcode.subsystems.antony.AntonyHardware
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmHardware
import org.firstinspires.ftc.teamcode.subsystems.depositClaw.DepositClawHardware
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveHardware
import org.firstinspires.ftc.teamcode.subsystems.extendo.ExtendoHardware
import org.firstinspires.ftc.teamcode.subsystems.intakeClaw.IntakeClawHardware
import org.firstinspires.ftc.teamcode.subsystems.lift.LiftHardware
import org.firstinspires.ftc.teamcode.subsystems.robot.Robot
import org.firstinspires.ftc.teamcode.subsystems.robot.Telemetry
import org.firstinspires.ftc.teamcode.subsystems.v4b.V4bHardware


@AntonyHardware.Attach
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
    AntonyHardware,
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