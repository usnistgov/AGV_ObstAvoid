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

import java.util.LinkedList;
import java.util.List;

/**
 * Class provides a way of creating a Cubic Spline from a list of Control Points.
 * 
 * @author Will Shackleford<shackle@nist.gov>
 */
public class BSplineCreator {
    

    public static List<Point2Dd> createBSpline(
            final int min_m, 
            final int max_m, 
            final double dist, 
            final List<? extends Point2Dd> plist) {        
        if(null == plist || plist.size() < 1) {
            return null;
        }
        if (plist.size() == 2) {
            LinkedList<Point2Dd> pa = new LinkedList<Point2Dd>();
            pa.add(plist.get(0));
            pa.add(plist.get(0));
            pa.add(plist.get(1));
            pa.add(plist.get(1));
            return createBSpline(max_m, min_m, dist, pa);
        }
        if (plist.size() == 3) {
            LinkedList<Point2Dd> pa = new LinkedList<Point2Dd>();
            pa.add(plist.get(0));
            pa.add(plist.get(1));
            pa.add(plist.get(1));
            pa.add(plist.get(2));
            return createBSpline(max_m, min_m, dist, pa);
        }
        if (plist == null || plist.size() < 4) {
            System.err.println("bsline() bad input = " + plist);
            return null;
        }
        final int n = plist.size();
        LinkedList<Point2Dd> p_out = new LinkedList<Point2Dd>();
        double xA, yA, xB, yB, xC, yC, xD, yD,
                a0, a1, a2, a3, b0, b1, b2, b3, x = 0, y = 0;
        for (int i = 0; i < n - 1; i++) {            
            Point2Dd pA = null;
            if (i <= 0) {
                pA = new Point2Dd(2*plist.get(0).x - plist.get(1).x,
                        2*plist.get(0).y - plist.get(1).y);
            } else {
                pA = plist.get(i - 1);
            }
            Point2Dd pB = plist.get(i);
            Point2Dd pC = plist.get(i + 1);
            Point2Dd pD = null;
            if (i >= n - 2) {
                pD = new Point2Dd(2*plist.get(i + 1).x - plist.get(i).x,
                        2*plist.get(i + 1).y - plist.get(i).y);
            } else {
                pD = plist.get(i + 2);
            }
//            System.out.println("pA = " + pA);
//            System.out.println("pB = " + pB);
//            System.out.println("pC = " + pC);
//            System.out.println("pD = " + pD);
            a3 = (-pA.x + 3 * (pB.x - pC.x) + pD.x) / 6;
            a2 = (pA.x - 2 * pB.x + pC.x) / 2;
            a1 = (pC.x - pA.x) / 2;
            a0 = (pA.x + 4 * pB.x + pC.x) / 6;
//            System.out.println("a0 = " + a0);
//            System.out.println("(a0+a1+a2+a3) = " + (a0 + a1 + a2 + a3));
            b3 = (-pA.y + 3 * (pB.y - pC.y) + pD.y) / 6;
            b2 = (pA.y - 2 * pB.y + pC.y) / 2;
            b1 = (pC.y - pA.y) / 2;
            b0 = (pA.y + 4 * pB.y + pC.y) / 6;
            int m = (int) ((pA.distance(pB) + pB.distance(pC) + pC.distance(pD)) / dist);
            if (m < min_m) {
                m = min_m;
            }
            if (m > max_m) {
                m = max_m;
            }
            for (int j = 1; j < m; j++) {                
                Point2Dd new_pt = new Point2Dd();
                double t = (double) j / (double) m;
                new_pt.x = x = ((a3 * t + a2) * t + a1) * t + a0;
                new_pt.y = y = ((b3 * t + b2) * t + b1) * t + b0;
                if (p_out.size() > 0 && p_out.getLast().x > new_pt.x) {
                    if(p_out.getLast().distance(new_pt) < dist/100f) {
                        continue;
                    }
//                    System.out.println("something bad??");
                }
                p_out.add(new_pt);
            }
        }
        return p_out;
    }
}
