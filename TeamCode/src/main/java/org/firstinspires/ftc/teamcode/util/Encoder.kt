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
    /**
 * Returns the current encoder position adjusted by the internal offset.
 *
 * The result represents the encoder's position relative to the configured starting pose.
 *
 * @return The adjusted encoder position.
 */
fun getPose() = motor.currentPosition+offset
    /**
     * Adjusts the encoder offset so that the reported position matches the specified value.
     *
     * @param value The desired encoder position to be reported by this instance.
     */
    fun setPose(value: Int) {
        offset = value - motor.currentPosition
    }

}