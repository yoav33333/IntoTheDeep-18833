package org.firstinspires.ftc.teamcode.opModes.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.util.ElapsedTime
import dev.frozenmilk.mercurial.commands.groups.Parallel
import dev.frozenmilk.mercurial.commands.groups.Sequential
import dev.frozenmilk.mercurial.commands.util.Wait
import org.firstinspires.ftc.teamcode.opModes.CommandOpMode
import org.firstinspires.ftc.teamcode.opModes.MegiddoOpMode
import org.firstinspires.ftc.teamcode.subsystems.followerSubsystem
@Autonomous
class Forward :MegiddoOpMode(){
    var delta = 0.0
    override fun myStart() {
        Sequential(followerSubsystem.forward.raceWith(Wait(1.0)),
            followerSubsystem.stop  )

    }

}