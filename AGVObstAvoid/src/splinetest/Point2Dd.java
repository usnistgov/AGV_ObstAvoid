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

import java.awt.geom.Point2D;

/**
 * Class extends the Point2D.Double representation of a line.
 * This allows this application to be refactored quickly if necessary.
 * It also provides better conversions too and from strings which are needed
 * for saving and restoring state.
 * 
 * @author Will Shackleford<shackle@nist.gov>
 */
public class Point2Dd extends Point2D.Double {
    
    public boolean selected=false;
    
    public Point2Dd() {
        super();
    }
    
    
    
    public Point2Dd(float _x, float _y) {
        super(_x,_y);
    }
    
    public Point2Dd(double _xd, double _yd) {
        super((float)_xd,(float)_yd);
    }
    
    public Point2Dd(Point2D.Double pt) {
        super((float)pt.x,(float)pt.y);
    }
    
    public Point2Dd(Point2D.Float pt) {
        super((float)pt.x,(float)pt.y);
    }
    
    public double dot(Point2D.Double pt) {
        return pt.x*x + pt.y*y;
    }
    
    public float mag() {
        return  (float) distance(0f,0f);
    }
    
    Point2Dd unit() {
        double _mag = mag();
        if(_mag <  1e-6f|| java.lang.Double.isInfinite(_mag) 
                || java.lang.Double.isNaN(_mag)) {
            throw new IllegalStateException("Can not compute unit vector when mag = "+_mag);
        }
        return new Point2Dd(x/_mag,y/_mag);
    }
    
    Point2Dd diff(Point2D.Double pt) {
        return new Point2Dd(x-pt.x,y-pt.y);
    }
    
    @Override
    public String toString() {
        return this.getClass().getName() + "["+x+","+y+"]";
    }
    
    public static Point2Dd valueOf(String s) {
        s=s.trim();
        int left_sq_paren_index = s.indexOf('[');
        if(left_sq_paren_index < 0) {
            return null;
        }
        int comma_index = s.indexOf(',');
        if(comma_index < left_sq_paren_index) {
            return null;
        }
        int right_sq_paren_index = s.indexOf(']');
        if(right_sq_paren_index < comma_index) {
            return null;
        }
        String xs = s.substring(left_sq_paren_index+1,comma_index).trim();
        String ys = s.substring(comma_index+1,right_sq_paren_index).trim();
        return new Point2Dd(java.lang.Double.valueOf(xs),java.lang.Double.valueOf(ys));
    }
    
    static public Point2Dd addPtDistAngle(Point2Dd pt, double dist, AngleD angle) {
        return new Point2Dd(
                (pt.x+dist*angle.cos()),
                (pt.y+dist*angle.sin()));
    }
    
    public boolean hasInfiniteOrNaN() {
        return java.lang.Double.isInfinite(x) || java.lang.Double.isInfinite(y)
                || java.lang.Double.isNaN(x) || java.lang.Double.isNaN(y);
    }
}
