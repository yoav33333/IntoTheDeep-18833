package org.firstinspires.ftc.teamcode.opModes

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.subsystems.Robot
import org.firstinspires.ftc.teamcode.subsystems.intakeSubsystem

@TeleOp
class RedTeleop: BadTeleop() {
    override fun setAlliance() {
        Robot.updateAllianceColor(intakeSubsystem.Color.RED)
    }
}