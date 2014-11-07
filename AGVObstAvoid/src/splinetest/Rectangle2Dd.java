/*
 * This is public domain software, however it is preferred
 * that the following disclaimers be attached.
 * 
 * Software Copywrite/Warranty Disclaimer
 * 
 * This software was developed at the National Institute of Standards and
 * Technology by employees of the Federal Government in the course of their
 * official duties. Pursuant to title 17 Section 105 of the United States
 * Code this software is not subject to copyright protection and is in the
 * public domain. NIST Real-Time Control System software is an experimental
 * system. NIST assumes no responsibility whatsoever for its use by other
 * parties, and makes no guarantees, expressed or implied, about its
 * quality, reliability, or any other characteristic. We would appreciate
 * acknowledgement if the software is used. This software can be
 * redistributed and/or modified freely provided that any derivative works
 * bear some notice that they are derived from it, and any modified
 * versions bear some notice that they have been modified.
 * 
 */

package splinetest;

import java.awt.geom.Rectangle2D;

/**
 *
 * @author William Shackleford<shackle@nist.gov>
 */
public class Rectangle2Dd extends Rectangle2D.Double {
    
    public Rectangle2Dd() {
        super();
    }
    
    public Rectangle2Dd(double _x, double _y, double _w, double _h) {
        super(_x,_y,_w,_h);
    }
    
    
    @Override
    public String toString() {
        return this.getClass().getName() + "["+x+","+y+","+this.width+","+this.height+"]";
    }
    
    public static Rectangle2Dd valueOf(String s) {
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
        if(comma2_index < left_sq_paren_index) {
            return null;
        }
        int comma3_index = s.indexOf(',',comma2_index+1);
        if(comma3_index < left_sq_paren_index) {
            return null;
        }
        int right_sq_paren_index = s.indexOf(']');
        if(right_sq_paren_index < comma_index) {
            return null;
        }
        String xs = s.substring(left_sq_paren_index+1,comma_index).trim();
        String ys = s.substring(comma_index+1,comma2_index).trim();
        String widths = s.substring(comma2_index+1,comma3_index).trim();
        String heights = s.substring(comma3_index+1,right_sq_paren_index).trim();
        return new Rectangle2Dd(java.lang.Double.valueOf(xs),
                        java.lang.Double.valueOf(ys),
                        java.lang.Double.valueOf(widths),
                        java.lang.Double.valueOf(heights)
                        );
    }
    
    
}
