package org.firstinspires.ftc.teamcode.subsystems.drive

import dev.frozenmilk.dairy.pasteurized.Pasteurized
import org.firstinspires.ftc.teamcode.subsystems.followerSubsystem.gamepad1

object DriveVariables {
    /*imu*/
    @JvmField var imuAngleOffset = 0.0
    /*drive*/
    @JvmField var maxPower = 1.0
    @JvmField var xSupplier = { -Pasteurized.gamepad1.leftStickX.state }
    @JvmField var ySupplier = { -(Pasteurized.gamepad1.leftStickY.state+Pasteurized.gamepad1.leftTrigger.state-Pasteurized.gamepad1.rightTrigger.state) }
    @JvmField var headingSupplier = { -(Pasteurized.gamepad1.rightStickX.state-0.5*(Pasteurized.gamepad2.leftTrigger.state-Pasteurized.gamepad2.rightTrigger.state)) }
    @JvmField var robotCentricSupplier = {Pasteurized.gamepad2.leftTrigger.state+ Pasteurized.gamepad2.rightTrigger.state>0.1 }
}