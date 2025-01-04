package PedroPathing.src.main.java.com.pedropathing.util;

public class Constants {
    public static Class<?> fConstants;
    public static Class<?> lConstants;

    public static void setConstants(Class<?> followerConstants, Class<?> localizerConstants) {
        fConstants = followerConstants;
        lConstants = localizerConstants;
        setup();
    }

    private static void setup() {
        try {
            Class.forName(fConstants.getName()); // This will trigger the static block for fConstants
            Class.forName(lConstants.getName()); // This will trigger the static block for lConstants
        } catch (ClassNotFoundException e) {
            e.printStackTrace();
        }
    }
}
