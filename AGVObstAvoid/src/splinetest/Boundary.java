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

 */


package splinetest;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;

/**
 * Class representing a boundary or line in a 2D-Map that vehicles may not cross.
 *
 * @author Will Shackleford<shackle@nist.gov>
 */
public class Boundary extends Line2Dd {

    public Boundary() {
        super();
    }

    public Boundary(double x1, double y1, double x2, double y2) {
        super(x1, y1, x2, y2);
    }
    
    public Boundary(Line2D.Double l2d) {
        super(l2d.getP1(),l2d.getP2());
    }

    public Boundary(double x1, double y1, double x2, double y2, boolean horz, boolean vert) {
        super(x1, y1, x2, y2);
        this.force_horizontal = horz;
        this.force_vertical = vert;
    }
    
    public boolean force_horizontal=false;
    public boolean force_vertical=false;

    public Boundary(Point2D p1, Point2D p2) {
        super(p1, p2);
    }

//    public double distance(Line2D.Double l) {
//        if (intersectsLine(l)) {
//            return 0;
//        }
//        Point2D lp1 = l.getP1();
//        Point2D lp2 = l.getP2();
//        Point2D tp1 = this.getP1();
//        Point2D tp2 = this.getP2();
//
//
//        double d = java.lang.Double.POSITIVE_INFINITY;
//        if(lp1.distance(tp1) )
//        (double) Math.min(this.ptSegDist(lp1),
//                this.ptSegDist(lp2));
//        d = (double) Math.min(l.ptSegDist(tp1), d);
//        d = (double) Math.min(l.ptSegDist(tp2), d);
//        return d;
//    }
    

    @Override
    public String toString() {
        return this.getClass().getName() + String.format("[[%.2f,%.2f],[%.2f,%.2f]]", x1, y1, x2, y2 );
    }

    public static Boundary valueOf(String s) {
        s = s.trim();
        int left_sq_paren_index = s.indexOf('[');
        if (left_sq_paren_index < 0) {
            return null;
        }
        int left_sq_paren_index2 = s.indexOf('[', left_sq_paren_index + 1);
        if (left_sq_paren_index2 < left_sq_paren_index) {
            return null;
        }
        int comma_index = s.indexOf(',', left_sq_paren_index2 + 1);
        if (comma_index < left_sq_paren_index) {
            return null;
        }
        int right_sq_paren_index = s.indexOf(']');
        if (right_sq_paren_index < comma_index) {
            return null;
        }
        int comma_index2 = s.indexOf(',', right_sq_paren_index + 1);
        if (comma_index2 < right_sq_paren_index) {
            return null;
        }
        int left_sq_paren_index3 = s.indexOf('[', comma_index2 + 1);
        if (left_sq_paren_index3 < comma_index2) {
            return null;
        }
        int comma_index3 = s.indexOf(',', left_sq_paren_index3 + 1);
        if (comma_index3 < left_sq_paren_index3) {
            return null;
        }
        int right_sq_paren_index2 = s.indexOf(']', comma_index3 + 1);
        if (right_sq_paren_index2 < comma_index3) {
            return null;
        }
        int right_sq_paren_index3 = s.indexOf(']', right_sq_paren_index2 + 1);
        if (right_sq_paren_index3 < right_sq_paren_index2) {
            return null;
        }
        String x1s = s.substring(left_sq_paren_index2 + 1, comma_index).trim();
        String y1s = s.substring(comma_index + 1, right_sq_paren_index).trim();
        String x2s = s.substring(left_sq_paren_index3 + 1, comma_index3).trim();
        String y2s = s.substring(comma_index3 + 1, right_sq_paren_index2).trim();
        return new Boundary(java.lang.Double.valueOf(x1s), java.lang.Double.valueOf(y1s),
                java.lang.Double.valueOf(x2s), java.lang.Double.valueOf(y2s));
    }
    public boolean P1Selected = false;
    public boolean P2Selected = false;

    @Override
    public Point2Dd getP1() {
        return new Point2Dd(x1, y1);
    }

    @Override
    public Point2Dd getP2() {
        return new Point2Dd(x2, y2);
    }
    
    boolean hasInfiniteOrNaN() {
        return getP1().hasInfiniteOrNaN() || getP2().hasInfiniteOrNaN();
    }
}
