package org.firstinspires.ftc.teamcode.util

import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.IMU

class SuperIMU(name: String) {
    private val imu= HardwareDevice(name, com.qualcomm.robotcore.hardware.IMU::class.java).get()
    var parameters: IMU.Parameters = IMU.Parameters(RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD))

    /**
     * Initializes the IMU with the current parameters and returns the IMU instance.
     *
     * @return The initialized IMU device.
     */
    fun get(): IMU {
        imu.initialize(parameters)
        return imu
    }

    /**
     * Sets the IMU's orientation on the robot for accurate sensor readings.
     *
     * Updates the IMU parameters with the specified robot orientation and returns this instance for method chaining.
     *
     * @param orientationOnRobot The physical orientation of the IMU on the robot.
     * @return This SuperIMU instance for chaining configuration calls.
     */
    fun setOrientationOnRobot(orientationOnRobot: RevHubOrientationOnRobot):SuperIMU {
        parameters.imuOrientationOnRobot = orientationOnRobot
        return this
    }

}