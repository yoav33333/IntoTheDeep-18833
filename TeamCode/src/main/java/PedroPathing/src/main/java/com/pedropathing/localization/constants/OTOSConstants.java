package PedroPathing.src.main.java.com.pedropathing.localization.constants;

import com.acmerobotics.roadrunner.ftc.SparkFunOTOSCorrected;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * This is the OTOSConstants class. It holds many constants and parameters for the OTOS Localizer.
 * @author Baron Henderson - 20077 The Indubitables
 * @version 1.0, 12/24/2024
 */

public class OTOSConstants {

    /** Whether to use the corrected OTOS class
     * @value Default Value: false */
    public static boolean useCorrectedOTOSClass = false;

    /** The name of the OTOS sensor in the hardware map
     * @value Default Value: "sensor_otos" */
    public static String hardwareMapName = "sensor_otos";

    /** The linear unit of the OTOS sensor
     * @value Default Value: DistanceUnit.INCH */
    public static DistanceUnit linearUnit = DistanceUnit.INCH;

    /** The angle unit of the OTOS sensor
     * @value Default Value: AngleUnit.RADIANS */
    public static AngleUnit angleUnit = AngleUnit.RADIANS;

    /** The offset of the OTOS sensor from the center of the robot
     * For the OTOS, left/right is the y axis and forward/backward is the x axis, with left being positive y and forward being positive x.
     * PI/2 radians is facing forward, and clockwise rotation is negative rotation.
     * @value Default Value: new Pose2D(0, 0, Math.PI / 2) */
    public static SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, Math.PI / 2);

    /** The linear scalar of the OTOS sensor
     * @value Default Value: 1.0 */
    public static double linearScalar = 1.0;

    /** The angular scalar of the OTOS sensor
     * @value Default Value: 1.0 */
    public static double angularScalar = 1.0;
}
