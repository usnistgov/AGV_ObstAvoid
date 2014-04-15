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
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

/**
 * PlannerPoint extends Point2Dd with data useful in the planning process or
 * in displays useful for debugging the planner.
 * 
 * @author Will Shackleford<shackle@nist.gov>
 */
class PlannerPoint extends Point2Dd {
    
    public PlannerPoint() {
        super();
    }
    
    public PlannerPoint(PlannerPoint pt) {
        super(pt.x, pt.y);
    }

    public PlannerPoint(Point2D.Double pt) {
        super(pt.x, pt.y);
    }

    public PlannerPoint(double _x, double _y) {
        super(_x, _y);
    }
    
    
    
    PlannerPoint unit() {
        double _mag = mag();
        return new PlannerPoint(x/_mag,y/_mag);
    }
    
    PlannerPoint diff(Point2D.Double pt) {
        return new PlannerPoint(x-pt.x,y-pt.y);
    }
    
    boolean opened = false;
    public PlannerPoint prev_pt=null;
    public PlannerPoint next_pt=null;
    public LinkedList<PlannerPoint> neighbors = null;
    public LinkedList<PlannerPoint> failed_neighbors = null;
    public LinkedList<PlannerPoint> potential_neighbors = null;
    public double pathFromStartDist = java.lang.Double.POSITIVE_INFINITY;
    public double distFromGoal;
    public boolean checked = false;
    public double pathEstimate = java.lang.Double.POSITIVE_INFINITY;
    Map<PlannerPoint,List<Line2Dd>> testedInLinesMap;
    Map<PlannerPoint,List<Line2Dd>> testedOutLinesMap;
    
    public double d1=0;
    
    
    public String toString() {
        StringBuffer sb = new StringBuffer();
        sb.append(super.toString());
        sb.append(",pathFromStartDist = ");
        sb.append(pathFromStartDist);
        sb.append(",distFromGoal = ");
        sb.append(distFromGoal);
//        if (null != neighbors) {
//            sb.append("\nneighbors:\n");
//            for (PlannerPoint pp : neighbors) {
//                sb.append(pp.tabbedToString(" ", 1));
//            }
//            sb.append("\n");
//        }
        return sb.toString();
    }

//    public String tabbedToString(String prefix, int level) {
//        StringBuffer sb = new StringBuffer();
//        sb.append(super.toString());
//        sb.append(",pathFromStartDist = ");
//        sb.append(pathFromStartDist);
//        sb.append(",distFromGoal = ");
//        sb.append(distFromGoal);
//        sb.append(",level = ");
//        sb.append(level);
//        if (null != neighbors && level < 5) {
//            sb.append("neighbors:\n");
//            sb.append(prefix);
//            for (PlannerPoint pp : neighbors) {
//                sb.append(prefix);
//                sb.append(pp.tabbedToString(prefix+" ", level+1));
//            }
//            sb.append("\n");
//        }
//        return sb.toString();
//    }
}
