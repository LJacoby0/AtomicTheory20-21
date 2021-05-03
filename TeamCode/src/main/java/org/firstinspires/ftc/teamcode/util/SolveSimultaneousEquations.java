package org.firstinspires.ftc.teamcode.util;

import org.jetbrains.annotations.Contract;
import org.jetbrains.annotations.NotNull;



public class SolveSimultaneousEquations {
    /**
     * Solve a system of two simultaneous equations
     * @param a the coefficient of the first term in the first equation
     * @param b the coefficient of the second term in the first equation
     * @param c the coefficient of the first term in the second equation
     * @param d the coefficient of the second term in the second equation
     * @param e what the first equation is equal to
     * @param f what the second equation is equal to
     * @return an array in the form [x, y]
     */
    @NotNull
    @Contract(value = "_, _, _, _, _, _ -> new", pure = true)
    public static double[] solveSimultaneousEquations(double a, double b, double c, double d, double e, double f) {
        double det = ((a) * (d) - (b) * (c));  //instead of 1/
        double x = ((d) * (e) - (b) * (f)) / det;
        double y = ((a) * (f) - (c) * (e)) / det;
        return new double[]{x, y};
    }
}
