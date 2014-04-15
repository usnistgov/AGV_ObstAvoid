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
 * Class extends the Point2Dd to represent a point with a radius around it 
 * which no vehicle should approach.
 * 
 * @author Will Shackleford<shackle@nist.gov>
 */
public class Obstacle  extends Point2Dd {
    
    public Obstacle() {
        super();
    }
    
    public Obstacle(double _x, double _y) {
        super(_x,_y);
    }
    
    public Obstacle(double _x, double _y, double _radius) {
        super(_x,_y);
        this.radius = _radius;
    }
    
    public Obstacle(Point2D.Double pt) {
        super(pt.x,pt.y);
    }
    
    public Obstacle(Point2D.Double pt, double _radius) {
        super(pt.x,pt.y);
        this.radius = _radius;
    }
    public double radius;
    
    @Override
    public String toString() {
        return this.getClass().getName() + String.format("[%.2f,%.2f,%.2f]",x,y,radius);
    }
    
    
    public static Obstacle valueOf(String s) {
        s=s.trim();
        int left_sq_paren_index = s.indexOf('[');
        if(left_sq_paren_index < 0) {
            return null;
        }
        int comma_index = s.indexOf(',');
        if(comma_index < left_sq_paren_index) {
            return null;
        }
        int comma2_index = s.indexOf(',',comma_index+1);
        if(comma2_index < comma_index) {
            return null;
        }
        int right_sq_paren_index = s.indexOf(']');
        if(right_sq_paren_index < comma_index) {
            return null;
        }
        String xs = s.substring(left_sq_paren_index+1,comma_index).trim();
        String ys = s.substring(comma_index+1,comma2_index).trim();
        String ws = s.substring(comma2_index+1,right_sq_paren_index).trim();
        return new Obstacle(java.lang.Double.valueOf(xs),java.lang.Double.valueOf(ys),java.lang.Double.valueOf(ws));
    }
}
