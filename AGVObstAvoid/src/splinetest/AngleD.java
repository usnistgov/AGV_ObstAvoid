/*
This is public domain software, however it is preferred
that the following disclaimers be attached.

Software Copywrite/Warranty Disclaimer

 This software was developed at the National Institute of Standards and
 Technology by employees of the Federal Government in the course of their
 official duties. Pursuant to title 17 Section 105 of the United States
 Code this software is not subject to copyright protection and is in the
 public domain. This software is experimental. NIST assumes no responsibility
 whatsoever for its use by other parties, and makes no guarantees, expressed
 or implied, about its quality, reliability, or any other characteristic.

 We would appreciate acknowledgement if the software is used. This software can
 be redistributed and/or modified freely provided that any derivative works
 bear some notice that they are derived from it, and any modified
 versions bear some notice that they have been modified.

tst change
 */

package splinetest;

/**
 * Wrapper class for angles. Angles are stored in radians and are forced to be
 * normalized from -pi to pi via the atan2() call in the setValue() method.
 *
 * @author Will Shackleford<shackle@nist.gov>
 */
public class AngleD implements Comparable<AngleD> {

    public AngleD() {
        this.setValue(0);
    }

    public AngleD(double _f) {
        this.setValue(_f);
    }
    
    public AngleD(AngleD _A) {
        this.setValue(_A.value);
    }
    private double value;

    /**
     * Get the value of value
     *
     * @return the value of value
     */
    public double getValue() {
        return value;
    }

    /**
     * Set the value of value
     *
     * @param value new value of value
     */
    public void setValue(double _value) {
        this.value = (Math.atan2(Math.sin(_value), Math.cos(_value)));
    }

    public double toDegrees() {
        return Math.toDegrees(value);
    }

    @Override
    public String toString() {
        return String.format("%.2f deg.", value*180.0/Math.PI);
    }

    public static AngleD valueOf(String s) {
        if (s.endsWith("deg.")) {
            s = s.substring(0, s.length() - 4);
        }
        s = s.trim();
        return new AngleD(Double.valueOf(s)*Math.PI/180.0);
    }

    public double cos() {
        return Math.cos(value);
    }

    public double sin() {
        return Math.sin(value);
    }
    
    public Point2Dd unit() {
        return new Point2Dd(this.cos(),this.sin());
    }

    public static AngleD diff(AngleD a1, AngleD a2) {
        return new AngleD(a1.value - a2.value);
    }

    public static AngleD add(AngleD a1, AngleD a2) {
        return new AngleD(a1.value + a2.value);
    }

    public static AngleD interpolate(AngleD a1, AngleD a2, double _s) {
        if (_s < 0 || _s > 1f) {
            throw new IllegalArgumentException("Anglef.interpolate(" + a1 + "," + a2 + "," + _s + ") : third argument must be between 0 and 1.");
        }
        double av1 = a1.value;
        double av2 = a2.value;
        if (av1 - av2 > Math.PI) {
            av2 += 2 * Math.PI;
        }
        if (av1 - av2 < -Math.PI) {
            av2 -= 2 * Math.PI;
        }
        return new AngleD(av1 * (1-_s) + av2 * _s);
    }

//    public static void main(String args[]) {
//        AngleD angle_inc = new AngleD(Math.PI/2);
//        for (double a1 = -2 * Math.PI; a1 < 2 * Math.PI; a1 += Math.PI / 36.0) {
//            AngleD A = new AngleD(a1);
//            AngleD B = AngleD.add(A, angle_inc);
//            System.out.println("A="+A+",B="+B+",interpolate(A,B,0.1)="+interpolate(A,B,0.1)+",interpolate(A,B,0.5)="+interpolate(A,B,0.5)+",interpolate(A,B,0.9)="+interpolate(A,B,0.9));
//        }
//    }

    @Override
    public int compareTo(AngleD o2) {
        return Double.compare(this.value, o2.value);
    }

    public int absCompare(AngleD o1, AngleD o2) {
        return Double.compare(Math.abs(o1.value), Math.abs(o2.value));
    }

    public int absCompareTo(AngleD o2) {
        return absCompare(this, o2);
    }
    
    
}
