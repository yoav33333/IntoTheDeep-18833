package org.firstinspires.ftc.teamcode.subsystems.drive

import dev.frozenmilk.dairy.core.wrapper.Wrapper
import dev.frozenmilk.mercurial.commands.Lambda
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveHardware.drive
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveHardware.setIMUHeading
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveVariables.maxPower
import java.util.function.DoubleSupplier

object DriveCommands {
    /**
         * Creates a command that drives the robot using supplied x, y, and rotation values.
         *
         * The command runs only while the operation mode is active. On initialization, it sets the maximum drive power to 1.0. During execution, it continuously updates the robot's movement based on the current values from the provided suppliers and the robot-centric mode flag.
         *
         * @param x Supplies the desired x-axis (strafe) input.
         * @param y Supplies the desired y-axis (forward/backward) input.
         * @param rotation Supplies the desired rotational input.
         * @param robotCentric Determines whether movement is robot-centric (default is field-centric).
         * @return A Lambda command that controls the robot's drive system.
         */
        fun driveCommand(x: DoubleSupplier, y: DoubleSupplier, rotation: DoubleSupplier, robotCentric: () -> Boolean = { false })
        =Lambda("DriveCommand")
        .setRunStates(Wrapper.OpModeState.ACTIVE)
        .setInit{maxPower = 1.0}
        .setExecute{drive(x.asDouble, y.asDouble, rotation.asDouble, robotCentric())}

    /**
         * Creates a command that sets the IMU heading to the specified angle during initialization.
         *
         * @param heading The desired heading angle in degrees to set the IMU to.
         * @return A Lambda command that sets the IMU heading when initialized.
         */
        fun setHeading(heading: Double) = Lambda("setHeading")
        .setRunStates(Wrapper.OpModeState.ACTIVE, Wrapper.OpModeState.INIT)
        .setInit { setIMUHeading(heading) }

    val resetHeading = setHeading(0.0)

}