package org.firstinspires.ftc.teamcode.util

import com.pedropathing.follower.Follower
import com.qualcomm.robotcore.hardware.HardwareMap

object FollowerInstance {
    lateinit var follower: Follower
    @JvmStatic
    fun getInstance(hardwareMap: HardwareMap): Follower {
        if (!::follower.isInitialized) {
            follower = Follower(hardwareMap)
        }
        return follower
    }
    @JvmStatic
    fun reset(hardwareMap: HardwareMap) {
        follower = Follower(hardwareMap)
    }
}