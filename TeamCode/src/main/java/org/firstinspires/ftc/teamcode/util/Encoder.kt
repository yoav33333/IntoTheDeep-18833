package org.firstinspires.ftc.teamcode.util

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import dev.frozenmilk.dairy.core.FeatureRegistrar


class Encoder(name: String,
              direction: DcMotorSimple.Direction = DcMotorSimple.Direction.FORWARD,
              startingPose: Int = 0) {
    private var motor: DcMotor =
        FeatureRegistrar.activeOpMode.hardwareMap.get(DcMotorEx::class.java, name)

    init {
        motor.direction = direction
    }

    var direction = direction
        set(value) {
            motor.direction = value
            field = value
        }
    var offset = startingPose
    fun getPose() = motor.currentPosition-offset
    fun setPose(value: Int) {
        offset -= motor.currentPosition - value
    }

}