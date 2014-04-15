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

import java.util.List;


/**
 * Class collects all input variables to the planner.
 * 
 * @author Will Shackleford<shackle@nist.gov>
 */
public class PlannerInput implements Cloneable{

    CarrierState start;
    CarrierState goal;
    List<Obstacle> obstacles;
    double veh_width;
    double front;
    double back;
    double path_uncertainty;
    boolean crab;
    double max_pt2pt_dist;
    List<Boundary> boundaries;
    double plannerResolution;
    int max_cntrl_pts;
    boolean reverse;
    double min_turn_radius;
    double planningHorizon;
    
    @Override
    public PlannerInput clone() {
        PlannerInput pi =  new PlannerInput();
        pi.start = this.start;
        pi.goal = this.goal;
        pi.obstacles = this.obstacles;
        pi.veh_width = this.veh_width;
        pi.front = this.front;
        pi.back = this.back;
        pi.path_uncertainty = this.path_uncertainty;
        pi.boundaries = this.boundaries;
        pi.crab = this.crab;
        pi.max_pt2pt_dist = this.max_pt2pt_dist;
        pi.plannerResolution = this.plannerResolution;
        pi.max_cntrl_pts = this.max_cntrl_pts;
        pi.reverse = this.reverse;
        pi.min_turn_radius = this.min_turn_radius;
        pi.planningHorizon = this.planningHorizon;
        return pi;
    }
    
    
}
