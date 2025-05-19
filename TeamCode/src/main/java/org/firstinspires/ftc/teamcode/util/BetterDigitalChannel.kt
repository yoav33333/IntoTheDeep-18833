package org.firstinspires.ftc.teamcode.util

import com.qualcomm.robotcore.hardware.DigitalChannel
import dev.frozenmilk.dairy.core.FeatureRegistrar

open class BetterDigitalChannel
/**
 * @param name the channel hardware map name
 */
constructor(
    open val name: String,
) : DigitalChannel by FeatureRegistrar.activeOpMode.hardwareMap.get(DigitalChannel::class.java, name)