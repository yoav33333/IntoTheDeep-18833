package org.firstinspires.ftc.teamcode.subsystems.drive

import com.acmerobotics.dashboard.config.Config
import dev.frozenmilk.dairy.pasteurized.Pasteurized
import java.util.function.BooleanSupplier
import java.util.function.DoubleSupplier
@Config
object DriveVariables {
    /*imu*/
    @JvmField var imuAngleOffset = 0.0
    @JvmField var startingAngle = 0.0
    /*drive*/
    @JvmField var maxPower = 1.0
    @JvmField var highGear = 1.0
    @JvmField var lowPGear = 0.4
    @JvmField var zaleskiSpeed = 0.5
    @JvmField var xSupplier = DoubleSupplier{ Pasteurized.gamepad1.leftStickX.state }
    @JvmField var ySupplier = DoubleSupplier{ (Pasteurized.gamepad1.leftStickY.state) }
    @JvmField var headingSupplier = DoubleSupplier{ (Pasteurized.gamepad1.rightStickX.state-zaleskiSpeed*(Pasteurized.gamepad2.leftTrigger.state-Pasteurized.gamepad2.rightTrigger.state)) }
    @JvmField var robotCentricSupplier = BooleanSupplier{false}
}