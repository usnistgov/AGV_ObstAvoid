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
    boolean use_static_planner_list;
    public double veh_width;
    public double front;
    public double back;
    public double path_uncertainty;
    boolean crab;
    Rectangle2Dd rectB;
    public double max_pt2pt_dist;
    List<Boundary> boundaries;
    public double plannerResolution;
    int max_cntrl_pts;
    boolean reverse;
    public double min_turn_radius;
    public double min_turn_radius_dot_limit;
    public double planningHorizon;
    public double segStartLength;
    public double max_turn_angle_degrees;
    
    public long create_planner_list_start_world_ms;
    public long create_planner_list_start_cpu_ns;
    public long create_planner_list_end_world_ms;
    public long create_planner_list_end_cpu_ns;
    
//    public double goal_start_surround_angle_inc;
//    public double goal_start_surround_dist_inc;
//    public double goal_start_surround_dist_max;
    
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
        pi.segStartLength = this.segStartLength;
        pi.max_turn_angle_degrees = this.max_turn_angle_degrees;
        this.min_turn_radius_dot_limit = Math.cos(Math.toRadians(this.max_turn_angle_degrees));
        pi.min_turn_radius_dot_limit = this.min_turn_radius_dot_limit;
        pi.create_planner_list_start_world_ms = this.create_planner_list_start_world_ms;
        pi.create_planner_list_start_cpu_ns = this.create_planner_list_start_cpu_ns;
        pi.create_planner_list_end_world_ms = this.create_planner_list_end_world_ms;
        pi.create_planner_list_end_cpu_ns = this.create_planner_list_end_cpu_ns;
//        pi.goal_start_surround_angle_inc = this.goal_start_surround_angle_inc;
//        pi.goal_start_surround_dist_inc = this.goal_start_surround_dist_inc;
//        pi.goal_start_surround_dist_max = this.goal_start_surround_dist_max;
        return pi;
    }
    
    
}
