package org.firstinspires.ftc.teamcode.util

import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.IMU

class SuperIMU(name: String) {
    private val imu= HardwareDevice(name, com.qualcomm.robotcore.hardware.IMU::class.java).get()
    var parameters: IMU.Parameters = IMU.Parameters(RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD))

    fun get(): IMU {
        imu.initialize(parameters)
        return imu
    }

    //builder
    fun setOrientationOnRobot(orientationOnRobot: RevHubOrientationOnRobot):SuperIMU {
        parameters.imuOrientationOnRobot = orientationOnRobot
        return this
    }

}