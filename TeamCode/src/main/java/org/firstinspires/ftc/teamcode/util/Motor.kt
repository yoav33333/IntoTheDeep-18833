package org.firstinspires.ftc.teamcode.util

import com.qualcomm.robotcore.hardware.DcMotorEx
import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx
import dev.frozenmilk.dairy.core.FeatureRegistrar
import kotlin.math.abs

open class Motor
/**
 * @param name the motor hardware map name
 */
constructor(
    open val name: String,
) : DcMotorEx by FeatureRegistrar.activeOpMode.hardwareMap.get(DcMotorEx::class.java, name)

