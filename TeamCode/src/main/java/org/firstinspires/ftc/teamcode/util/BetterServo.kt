package org.firstinspires.ftc.teamcode.util

import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.Servo
import dev.frozenmilk.dairy.cachinghardware.CachingServo
import dev.frozenmilk.dairy.core.FeatureRegistrar
import dev.frozenmilk.dairy.core.util.OpModeLazyCell

open class BetterServo
/**
 * @param name the servo hardware map name
 */
constructor(
    open val name: String,
) : Servo by FeatureRegistrar.activeOpMode.hardwareMap.get(Servo::class.java, name)