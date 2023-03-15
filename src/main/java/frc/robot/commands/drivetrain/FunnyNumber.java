package frc.robot.commands.drivetrain;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.function.DoubleConsumer;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Registers a number on the smart dashboard
 */
public class FunnyNumber {
    private static final HashMap<String, Pair<Double, DoubleConsumer>> numbers = new HashMap<>();
    private static void insertIfEmpty(String name, double defaultValue, DoubleConsumer setter) {
        if (!numbers.containsKey(name)) {
            numbers.put(name, Pair.of(defaultValue, setter)); 
            SmartDashboard.putNumber(name, defaultValue);
        }
    }
    private static double pollDashboard(String name, double val, DoubleConsumer setter) {
        var got = SmartDashboard.getNumber(name, val);
        if (got != val) {
            if (setter != null) setter.accept(got);
            numbers.put(name, Pair.of(got, setter));    // will overwrite previous setter
        }
        return got;
    }
    public static double funnynumber(String name, double defaultValue, DoubleConsumer setter) {
        insertIfEmpty(name, defaultValue, setter);
        var n = numbers.get(name);
        return pollDashboard(name, n.getFirst(), n.getSecond());
    }
    public static double funnynumber(String name, double defaultValue) {
        insertIfEmpty(name, defaultValue, null);
        var n = numbers.get(name);
        return pollDashboard(name, n.getFirst(), n.getSecond());
    }

    // // MAKE SURE YOU CALL THIS FUNCTION IN PERIODIC
    // // MAKE SURE YOU CALL THIS FUNCTION IN PERIODIC
    // // MAKE SURE YOU CALL THIS FUNCTION IN PERIODIC
    // public static void updateValues() {
    //     for (var )
    //     for (var fn : numbers) {
    //         fn.update();
    //     }
    // }
    // public FunnyNumber(String name, double defaultValue, DoubleConsumer setter) {
    //     this.name = name;
    //     this.val = defaultValue;
    //     this.setter = setter;
    //     SmartDashboard.putNumber(name, defaultValue);
    // }
    // public void update() {
    //     double got = SmartDashboard.getNumber(name, val);
    //     if (got != val && this.setter != null) {
    //         this.setter.accept(got);
    //     }
    //     val = got;
    // }
    // public double get() {
    //     return val;
    // }
    public static double sdlog(String name, double value) {
        SmartDashboard.putNumber(name, value);
        return value;
    }
}
