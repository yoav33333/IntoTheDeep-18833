package PedroPathing.src.main.java.com.pedropathing.follower;


import com.acmerobotics.dashboard.config.Config;
import PedroPathing.src.main.java.com.pedropathing.localization.Localizers;
import PedroPathing.src.main.java.com.pedropathing.pathgen.MathFunctions;
import PedroPathing.src.main.java.com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import PedroPathing.src.main.java.com.pedropathing.pathgen.Vector;
import PedroPathing.src.main.java.com.pedropathing.util.CustomFilteredPIDFCoefficients;
import PedroPathing.src.main.java.com.pedropathing.util.CustomPIDFCoefficients;
import PedroPathing.src.main.java.com.pedropathing.util.KalmanFilterParameters;

/**
 * This is the FollowerConstants class. It holds many constants and parameters for various parts of
 * the Follower. This is here to allow for easier tuning of Pedro Pathing, as well as concentrate
 * everything tunable for the Paths themselves in one place.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @author Baron Henderson - 20077 The Indubitables
 * @version 1.0, 3/4/2024
 */

@Config
public class FollowerConstants {

    /** The Localizer that the Follower & Pose Updater will use
     * @value Default Value: Localizers.THREE_WHEEL */
    public static Localizers localizers = Localizers.THREE_WHEEL;

    /** The name of the left front motor
     * @value Default Value: "leftFront" */
    public static String leftFrontMotorName = "leftFront";

    /** The name of the left rear motor
     * @value Default Value: "leftRear" */
    public static String leftRearMotorName = "leftRear";

    /** The name of the right front motor
     * @value Default Value: "rightFront" */
    public static String rightFrontMotorName = "rightFront";

    /** The name of the right rear motor
     * @value Default Value: "rightRear" */
    public static String rightRearMotorName = "rightRear";

    /** The direction of the left front motor
     * @value Default Value: DcMotorSimple.Direction.REVERSE */
    public static DcMotorSimple.Direction leftFrontMotorDirection = DcMotorSimple.Direction.REVERSE;

    /** The direction of the right front motor
     * @value Default Value: DcMotorSimple.Direction.REVERSE */
    public static DcMotorSimple.Direction rightFrontMotorDirection = DcMotorSimple.Direction.REVERSE;

    /** The direction of the left rear motor
     * @value Default Value: DcMotorSimple.Direction.FORWARD */
    public static DcMotorSimple.Direction leftRearMotorDirection = DcMotorSimple.Direction.FORWARD;

    /** The direction of the right rear motor
     * @value Default Value: DcMotorSimple.Direction.FORWARD */
    public static DcMotorSimple.Direction rightRearMotorDirection = DcMotorSimple.Direction.FORWARD;

    /** The motor caching threshold
     * @value Default Value: 0.01 */
    public static double motorCachingThreshold = 0.01;

    /** The Forward Velocity of the Robot - Different for each robot
     * @value Default Value: 81.34056 */
    public static double xMovement = 81.34056;

    /** The Lateral Velocity of the Robot - Different for each robot
     * @value Default Value: 65.43028 */
    public static double yMovement = 65.43028;


    private static double[] convertToPolar = Point.cartesianToPolar(xMovement, -yMovement);

    /** The actual drive vector for the front left wheel, if the robot is facing a heading of 0 radians with the wheel centered at (0,0)
     * @value Default Value: new Vector(convertToPolar[0], convertToPolar[1])
     * @implNote This vector should not be changed, but only accessed.
     */
    public static Vector frontLeftVector = MathFunctions.normalizeVector(new Vector(convertToPolar[0], convertToPolar[1]));

    /** Global Max Power (can be overridden, just a default)
     * @value Default Value: 1 */
    public static double maxPower = 1;


    /** Translational PIDF coefficients (don't use integral)
     * @value Default Value: new CustomPIDFCoefficients(0.1,0,0,0); */
    public static CustomPIDFCoefficients translationalPIDFCoefficients = new CustomPIDFCoefficients(
            0.1,
            0,
            0,
            0);

    /** Translational Integral
     * @value Default Value: new CustomPIDFCoefficients(0,0,0,0); */
    public static CustomPIDFCoefficients translationalIntegral = new CustomPIDFCoefficients(
            0,
            0,
            0,
            0);

    /** Feed forward constant added on to the translational PIDF
     * @value Default Value: 0.015 */
    public static double translationalPIDFFeedForward = 0.015;


    /** Heading error PIDF coefficients
     * @value Default Value: new CustomPIDFCoefficients(1,0,0,0); */
    public static CustomPIDFCoefficients headingPIDFCoefficients = new CustomPIDFCoefficients(
            1,
            0,
            0,
            0);

    /** Feed forward constant added on to the heading PIDF
     * @value Default Value: 0.01 */
    public static double headingPIDFFeedForward = 0.01;


    /** Drive PIDF coefficients
     * @value Default Value: new CustomFilteredPIDFCoefficients(0.025,0,0.00001,0.6,0); */
    public static CustomFilteredPIDFCoefficients drivePIDFCoefficients = new CustomFilteredPIDFCoefficients(
            0.025,
            0,
            0.00001,
            0.6,
            0);

    /** Feed forward constant added on to the drive PIDF
     * @value Default Value: 0.01 */
    public static double drivePIDFFeedForward = 0.01;

    /** Kalman filter parameters for the drive error Kalman filter
     * @value Default Value: new KalmanFilterParameters(6,1); */
    public static KalmanFilterParameters driveKalmanFilterParameters = new KalmanFilterParameters(
            6,
            1);


    /** Mass of robot in kilograms
     * @value Default Value: 10.65942 */
    public static double mass = 10.65942;

    /** Centripetal force to power scaling
     * @value Default Value: 0.0005 */
    public static double centripetalScaling = 0.0005;


    /** Acceleration of the drivetrain when power is cut in inches/second^2 (should be negative)
     * if not negative, then the robot thinks that its going to go faster under 0 power
     * @value Default Value: -34.62719
     * @implNote This value is found via 'ForwardZeroPowerAccelerationTuner'*/
    public static double forwardZeroPowerAcceleration = -34.62719;

    /** Acceleration of the drivetrain when power is cut in inches/second^2 (should be negative)
     * if not negative, then the robot thinks that its going to go faster under 0 power
     * @value Default Value: -78.15554
     * @implNote This value is found via 'LateralZeroPowerAccelerationTuner'*/
    public static double lateralZeroPowerAcceleration = -78.15554;


    /** A multiplier for the zero power acceleration to change the speed the robot decelerates at
     * the end of paths.
     * Increasing this will cause the robot to try to decelerate faster, at the risk of overshoots
     * or localization slippage.
     * Decreasing this will cause the deceleration at the end of the Path to be slower, making the
     * robot slower but reducing risk of end-of-path overshoots or localization slippage.
     * This can be set individually for each Path, but this is the default.
     * @value Default Value: 4
     */
    public static double zeroPowerAccelerationMultiplier = 4;


    /** When the robot is at the end of its current Path or PathChain and the velocity goes below
     * this value, then end the Path. This is in inches/second.
     * This can be custom set for each Path.
     * @value Default Value: 0.1 */
    public static double pathEndVelocityConstraint = 0.1;

    /** When the robot is at the end of its current Path or PathChain and the translational error
     * goes below this value, then end the Path. This is in inches.
     * This can be custom set for each Path.
     * @value Default Value: 0.1 */
    public static double pathEndTranslationalConstraint = 0.1;

    /** When the robot is at the end of its current Path or PathChain and the heading error goes
     * below this value, then end the Path. This is in radians.
     * This can be custom set for each Path.
     * @value Default Value: 0.007 */
    public static double pathEndHeadingConstraint = 0.007;

    /** When the t-value of the closest point to the robot on the Path is greater than this value,
     * then the Path is considered at its end.
     * This can be custom set for each Path.
     * @value Default Value: 0.995 */
    public static double pathEndTValueConstraint = 0.995;

    /** When the Path is considered at its end parametrically, then the Follower has this many
     * milliseconds to further correct by default.
     * This can be custom set for each Path.
     * @value Default Value: 500 */
    public static double pathEndTimeoutConstraint = 500;

    /** This is how many steps the BezierCurve class uses to approximate the length of a BezierCurve.
     * @see #BEZIER_CURVE_BINARY_STEP_LIMIT
     * @value Default Value: 1000 */
    public static int APPROXIMATION_STEPS = 1000;

    /** This scales the translational error correction power when the Follower is holding a Point.
     * @value Default Value: 0.45 */
    public static double holdPointTranslationalScaling = 0.45;

    /** This scales the heading error correction power when the Follower is holding a Point.
     * @value Default Value: 0.35 */
    public static double holdPointHeadingScaling = 0.35;

    /** This is the number of times the velocity is recorded for averaging when approximating a first
     * and second derivative for on the fly centripetal correction. The velocity is calculated using
     * half of this number of samples, and the acceleration uses all of this number of samples.
     * @see #centripetalScaling
     * @value Default Value: 8 */
    public static int AVERAGED_VELOCITY_SAMPLE_NUMBER = 8;

    /** This is the number of steps the binary search for closest point uses. More steps is more
     * accuracy, and this increases at an exponential rate. However, more steps also does take more
     * time.
     * @see #APPROXIMATION_STEPS
     * @value Default Value: 10 */
    public static int BEZIER_CURVE_BINARY_STEP_LIMIT = 10;

    /** This activates/deactivates the secondary translational PIDF. It takes over at a certain translational error
     * @see #translationalPIDFSwitch
     * @value Default Value: false */
    public static boolean useSecondaryTranslationalPID = false;

    /** Use the secondary heading PIDF. It takes over at a certain heading error
     * @see #headingPIDFSwitch
     * @value Default Value: false */
    public static boolean useSecondaryHeadingPID = false;

    /** Use the secondary drive PIDF. It takes over at a certain drive error
     * @see #drivePIDFSwitch
     * @value Default Value: false */
    public static boolean useSecondaryDrivePID = false;

    /** The limit at which the translational PIDF switches between the main and secondary translational PIDFs,
     * if the secondary PID is active.
     * @see #useSecondaryTranslationalPID
     * @value Default Value: 3 */
    public static double translationalPIDFSwitch = 3;

    /** Secondary translational PIDF coefficients (don't use integral).
     * @see #useSecondaryTranslationalPID
     * @value Default Value: new CustomPIDFCoefficients(0.3, 0, 0.01, 0) */
    public static CustomPIDFCoefficients secondaryTranslationalPIDFCoefficients = new CustomPIDFCoefficients(
            0.3,
            0,
            0.01,
            0);

    /** Secondary translational Integral value.
     * @see #useSecondaryTranslationalPID
     * @value Default Value: new CustomPIDFCoefficients(0, 0, 0, 0) */
    public static CustomPIDFCoefficients secondaryTranslationalIntegral = new CustomPIDFCoefficients(
            0,
            0,
            0,
            0);

    /** Feed forward constant added on to the small translational PIDF.
     * @see #useSecondaryTranslationalPID
     * @see #secondaryTranslationalPIDFCoefficients
     * @value Default Value: 0.015 */
    public static double secondaryTranslationalPIDFFeedForward = 0.015;

    /** The limit at which the heading PIDF switches between the main and secondary heading PIDFs.
     * @see #useSecondaryHeadingPID
     * @value Default Value: Math.PI / 20 */
    public static double headingPIDFSwitch = Math.PI / 20;

    /** Secondary heading error PIDF coefficients.
     * @see #useSecondaryHeadingPID
     * @value Default Value: new CustomPIDFCoefficients(5, 0, 0.08, 0) */
    public static CustomPIDFCoefficients secondaryHeadingPIDFCoefficients = new CustomPIDFCoefficients(
            5,
            0,
            0.08,
            0);

    /** Feed forward constant added on to the secondary heading PIDF.
     * @see #useSecondaryHeadingPID
     * @see #secondaryHeadingPIDFCoefficients
     * @value Default Value: 0.01 */
    public static double secondaryHeadingPIDFFeedForward = 0.01;

    /** The limit at which the heading PIDF switches between the main and secondary drive PIDFs.
     * @see #useSecondaryDrivePID
     * @value Default Value: 20 */
    public static double drivePIDFSwitch = 20;

    /** Secondary drive PIDF coefficients.
     * @see #useSecondaryDrivePID
     * @value Default Value: new CustomFilteredPIDFCoefficients(0.02, 0, 0.000005, 0.6, 0) */
    public static CustomFilteredPIDFCoefficients secondaryDrivePIDFCoefficients = new CustomFilteredPIDFCoefficients(
            0.02,
            0,
            0.000005,
            0.6,
            0);

    /** Feed forward constant added on to the secondary drive PIDF.
     * @see #useSecondaryDrivePID
     * @value Default Value: 0.01 */
    public static double secondaryDrivePIDFFeedForward = 0.01;

    /** Use break mode for the drive motors in teleop
     * @value Default Value: false */
    public static boolean useBrakeModeInTeleOp = false;

    /** A Value solely used to debug the constantsUser */
    public static double constantsDebug = 0;
}
