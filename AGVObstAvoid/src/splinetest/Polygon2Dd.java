 
package splinetest;

import java.awt.geom.Path2D;
import java.awt.geom.PathIterator;
import java.awt.geom.Point2D;

/**
 *
 * @author Will Shackleford<shackle@nist.gov>
 */


public class Polygon2Dd extends Path2D.Double{
    
    public void addPoint(Point2D.Double pt) {
        if(this.getPathIterator(null).isDone()) {
            this.moveTo(pt.x, pt.y);
        } else {
            this.lineTo(pt.x,pt.y);
        }
    }
    
    @Override
    public String toString() {
        StringBuffer sb = new StringBuffer();
        sb.append(this.getClass().getName());
        sb.append("[");
        PathIterator pi = this.getPathIterator(null);
        double values[] = new double[6];
        boolean first = true;
        while(!pi.isDone()) {
              int type = pi.currentSegment(values);
              System.out.println("type = " + type);
              switch(type) {
                  case PathIterator.SEG_MOVETO:
                      if(!first) {
                          sb.append(",");
                      }
                      sb.append("[");
                      sb.append(values[0]);
                      sb.append(",");
                      sb.append(values[1]);
                      sb.append("]");
                      first = false;
                      break;
                  case PathIterator.SEG_LINETO:
                      if(!first) {
                          sb.append(",");
                      }
                      sb.append("[");
                      sb.append(values[0]);
                      sb.append(",");
                      sb.append(values[1]);
                      sb.append("]");
                      first = false;
                      break;
                      
                  default:
                      break;
              }
              pi.next();
        }
        sb.append("]");
        return sb.toString();
    }
    
    public static Polygon2Dd valueOf(String s) {
        Polygon2Dd poly = new Polygon2Dd();
        int left_sq_paren_index = s.indexOf('[');
        if(left_sq_paren_index < 0) {
            return null;
        }
        int sq_count = 0;
        int end_index = -1;
        for(int i = left_sq_paren_index+1; i < s.length(); i++) {
            switch(s.charAt(i)) {
                case '[':
                    sq_count++;
                    break;
                    
                case ']': 
                    end_index = i;
                    sq_count--;
                    break;
            }
            if(sq_count < 0) {
                break;
            }
        }
        if(end_index < 0 || sq_count > 0) {
            return null;
        }
        s = s.substring(left_sq_paren_index+1, end_index);
        s=s.trim();
        left_sq_paren_index = s.indexOf('[');
        int right_sq_paren_index = s.indexOf(']',left_sq_paren_index+1);
        while(left_sq_paren_index >= 0 && right_sq_paren_index >= 0) {
            String spt = s.substring(left_sq_paren_index+1, right_sq_paren_index);
            String vs[] =spt.split("[,]");
            if(vs.length >= 2) {
                poly.addPoint(new Point2Dd(java.lang.Double.valueOf(vs[0]), java.lang.Double.valueOf(vs[1])));
            }
            left_sq_paren_index = s.indexOf('[',right_sq_paren_index+1);
            right_sq_paren_index = s.indexOf(']',left_sq_paren_index+1);
        }
        poly.closePath();
        return poly;
    }
    
    static public Polygon2Dd valueOf(Point2Dd[] pta) {
        Polygon2Dd poly = new Polygon2Dd();
        for(Point2Dd pt : pta) {
            poly.addPoint(pt);
        }
        poly.closePath();
        return poly;
    }
    
    static public void main(String args[]) {
        Polygon2Dd poly = Polygon2Dd.valueOf("[[1,2],[3,4],[5,6]]");
        System.out.println("poly = " + poly);
    }
}
