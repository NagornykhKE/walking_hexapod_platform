#!/usr/bin/env python3

# license removed for brevity
import math
import re

import xml.etree.ElementTree as ET

import rospy

from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist

def angle_diff(a1, a2):
    """ difference between 2 angles """
    diff = a1 - a2
    return (diff + math.pi) % (2*math.pi) - math.pi

def gen_walking_template(dx, dy, height_down, height_up,
                         step_dist, step_dy):
    p1 = (dx, dy, -height_down)
    p2 = (dx - step_dist*0.5, dy - step_dy*0.5, -height_down)
    p3 = (dx - step_dist*0.5, dy - step_dy*0.5, -height_up)
    p4 = (dx + step_dist*0.5, dy + step_dy*0.5, -height_up)
    p5 = (dx + step_dist*0.5, dy + step_dy*0.5, -height_down)
    return [p1, p2, p2, p3, p4, p5, p5, p1]

# class for a single jointgenerate_get_up_animation
class JointDescr(object):
    def __init__(self, pos, side, num):
        # consts
        self.pos = pos
        self.side = side
        self.num = num
        self.joint_name = self.get_joint_name()
        
        # state
        self.goal = None
        self.curr_pos = 0.
        
        self.pub = rospy.Publisher(self.get_cmd_topic_name(), Float64, queue_size=10)
        
    def get_cmd_topic_name(self):
        return '{}_{}{}_position_controller/command'.format(self.side, self.pos, self.num)
    
    def get_joint_name(self):
        return '{}_{}{}_joint'.format(self.side, self.pos, self.num)
    
    def set_goal_pos(self, pos, duration, curr_time):
        self.goal = (self.curr_pos, pos, duration, curr_time)

    def update_current_pos(self, j_st_msg):
        try:
            ind = j_st_msg.name.index(self.joint_name)
        except:
            rospy.logerr('ant_motion_controller: data about joint {} not found in joint_states'
                         .format(self.joint_name))
            return
        self.curr_pos = j_st_msg.position[ind]

    def process(self, curr_time):
        if self.goal is None:
            return
        # get goal
        start_pos, goal_pos, duration, start_time = self.goal
        # process instant command separately
        if abs(duration) < 1e-2:
            pos = goal_pos
        else:
            time_from_start = (curr_time - start_time).to_sec()
            if time_from_start < duration - 1e-2: # process separately movements
                                                  # that are almost finished
                pos = time_from_start / duration * (goal_pos - start_pos) + start_pos
            else:
                pos = goal_pos
                
        # send message
        msg = Float64()
        msg.data = pos
        self.pub.publish(msg)
        

class LimbAnimation(object):
    
    def __init__(self, name, moments, poses):
        assert(len(moments) == len(poses))
        assert(len(moments) > 2)
        # TODO add extra checks
        self.name = name
        self.moments = moments
        self.source_poses = poses
        self.actual_poses = poses[:]
    
    def get_pose(self, phase):
        if phase <= self.moments[0]:
            return self.actual_poses[0]
        if phase >= self.get_duration():
            return self.actual_poses[-1]
        
        for i in range(len(self.moments) - 1):
            if self.moments[i+1] > phase:
                # interpolate position from moments i and i+1
                coeff = (phase - self.moments[i]) / (self.moments[i+1] - self.moments[i])
                return [p1 * (1 - coeff) + p2*coeff 
                        for p1,p2 in zip(self.actual_poses[i], self.actual_poses[i+1])]
        return self.actual_poses[-1]
    
    def get_duration(self):
        return self.moments[-1]
    
class LimbDescr(object):
    def __init__(self, pos, side, j_num, geom_params):
        self.pos = pos
        self.side = side
        self.geom_params = geom_params
        
        # create joints
        self.joints = [JointDescr(pos, side, i)
                       for i in range(1, j_num + 1)]
        self.animations = {}
    
    def set_anim_velocity(self, anim_name, vx, vy, wz):
        if anim_name not in self.animations.keys():
            rospy.logerr('ant_motion_controller: animation {} not found in limb {}_{}'
                         .format(anim_name, self.side, self.pos))
            return
        self.animations[name].set_velocity(vx, vy, wz)
    
    def set_limb_pos(pos_list, duration, curr_time):
        for jnt,pos in zip(self.joints,pos_list):
            jnt.set_goal_pos(pos, duration, curr_time)
        
    def process(self, curr_time):
        for jnt in self.joints:
            jnt.process(curr_time)
            
    def add_animation_description(self, name, moments, poses):
        self.animations[name] = LimbAnimation(name, moments, poses)
        
    def update_current_pos(self, j_st_msg):
        for jnt in self.joints:
            jnt.update_current_pos(j_st_msg)

    def execute_animation(self, name, start_moment, time_from_start):
        # get animation description
        if name not in self.animations.keys():
            rospy.logerr('ant_motion_controller: data abount animation {} is absent in limb {}_{}'
                         .format(name, self.side, self.pos))
            return
        animation = self.animations[name]
        phase = (time_from_start - start_moment) % animation.get_duration()
        dx, dy, z = animation.get_pose(phase)
        goal_pose = self.ik_local(dx, dy, z)
        # send goals to joints
        for i,jd in enumerate(self.joints):
            jd.set_goal_pos(goal_pose[i],0,None)
    
    def ik_local(self, dx, dy, z, debug_print = False):
        """ calculate angles for inverse kinematics
        with positions relative to base of the limb """
        _,_,ori,l1,l2,l3 = self.geom_params
        ori_target = math.atan2(dy,dx)
        # angle for 1st joint
        j1 = angle_diff(ori_target, ori)
        rad = math.hypot(dx, dy) - l1
        # solve ik task: side1 = l2, side2 = l3, side3:
        side3 = math.hypot(rad, z)
        error_found = False
        if debug_print:
            rospy.logerr('='*30 + '\nori: {:.3f}, j1: {:.3f}, rad: {:.3f}, l2: {:.3f}, l3: {:.3f}'
                         .format(ori_target, j1, rad, l2, l3))
        try:
            j2 = math.acos((side3**2 + l2**2 - l3**2) / (2*side3*l2)) + math.atan2(z, rad)
        except ValueError:
            rospy.logerr('ant_motion_controller: too big angle in j2 for limb ({},{}) in position {:.5f}, {:.5f}, {:.5f}'
                         .format(self.pos, self.side, dx, dy, z))
            j2 = 0.
            error_found = True
        try:
            j3 = math.acos((l2**2 + l3**2 - side3**2) / (2*l2*l3))
        except ValueError:
            rospy.logerr('ant_motion_controller: too big angle in j3 for limb ({},{}) in position {:.5f}, {:.5f}, {:.5f}'
                         .format(self.pos, self.side, dx, dy, z))
            j3 = 0
            error_found = True
        j4 = 0
        return (j1,-j2,math.pi - j3,j4)

    def fk_local(self, a1, a2, a3, a4):
        """ forward kinematics """
        _,_,ori,l1,l2,l3 = self.geom_params
        # vector of first part
        v1 = (math.cos(ori + a1), math.sin(ori+a1), 0.)
        p1 = (v1[0]*l1, v1[1]*l1, v1[2]*l1)
        # vector of second part
        c2 = math.cos(a2)
        s2 = math.sin(a2)
        v2 = (v1[0], v1[1]*c2 - v1[2]*s2, v1[1]*s2 + v1[2]*c2)
        p2 = (p1[0] + v2[0]*l2, p1[1] + v2[1]*l2, p1[2] + v2[2]*l2)
        # vector of third part
        c3 = math.cos(a3)
        s3 = math.sin(a3)
        v3 = (v2[0], v2[1]*c3 - v2[2]*s3, v2[1]*s3 + v2[2]*c3)
        p3 = (p2[0] + v3[0]*l3, p2[1] + v3[1]*l3, p2[2] + v3[2]*l3)
        # vector of fourth part
        return p3

    def get_animation_duration(self, name):
        if name in self.animations.keys():
            return self.animations[name].get_duration()
        else:
            return 0.

re_limb_and_pos = re.compile('([a-zA-Z]+)_([a-zA-Z]+)([0-9]+)_link')
def get_limb_and_pos_from_str(limb_pos_str):
    """ parse link name to get limb and position """
    m = re_limb_and_pos.match(limb_pos_str)
    if m is None:
        rospy.logwarn('ant_motion_controller: unknown limb and pos of link {}'
                      .format(limb_pos_str))
        return (None, None, None)
    else:
        return (m.group(1), m.group(2), int(m.group(3)))

def get_floats_from_string(input_str):
    try:
        return [float(v) for v in input_str.split(' ')]
    except ValueError:
        rospy.logwarn('ant_motion_controller: wrong format of float numbers: "{}"'
                      .format(input_str))
        return []



class AntDescr(object):
    def __init__(self, descr_str, step_dist, cycle_time):
        
        # load data from description
        descr = self._load_description_from_xml_model(descr_str)
        self._generate_limbs(*descr)
        
        # initialize limbs
        #possible_pos = ['forward', 'middle', 'backward']
        #possible_sides = ['left', 'right']
        #self.limbs = {(p,s) : LimbDescr(p,s,4)
        #    for p in possible_pos
        #    for s in possible_sides}
        
        # constants
        self.leg_y = 0.25
        self.body_height_down = 0.3
        self.body_height_up = 0.2
        self.step_dist = step_dist # 0.0003
        self.cycle_time = cycle_time
        self.step_diff = 0.
        
        self.dx_zero = {'forward': 0.03,
                        'middle': 0.,
                        'backward': 0.03}
        self.dy_zero = {'forward': 0.01,
                        'middle': 0.03,
                        'backward': 0.2}
        
        # state variables
        self.anim_plan = {}
        self.animation_start_time = None
        
        # subscriber to update joint states
        self.j_st_sub = rospy.Subscriber('joint_states', JointState, self.cb_j_st)
        
        # subscribe to velocity command
        self.cmd_vel_sub = rospy.Subscriber('cmd_vel', Twist, self.cb_cmd_vel)
        
        # timer
        self.anim_timer = rospy.Timer(rospy.Duration(0.05), self.cb_anim_timer)

    def cb_cmd_vel(self, msg):
        """ change animation according to velocities """
        for (pos, side), limb in self.limbs.items():
            # calculate disps
            dx = self.dx_zero[pos]
            dy = self.dy_zero[pos] + self.leg_y
            if side == 'right':
                dy = -dy
            # shifts
            sh_x = msg.linear.x * self.step_dist
            sh_y1 = msg.linear.y * self.step_dist
            sh_y2 = msg.angular.z * self.step_dist
            if pos == 'middle':
                sh_y2 = 0.
            elif pos == 'backward':
                sh_y2 = -sh_y2
            # rotation of legs
            sh_x2 = msg.angular.z * self.step_dist if side == 'right' else -msg.angular.z * self.step_dist
            # get positions
            poses = gen_walking_template(dx, dy,
                                         self.body_height_down, self.body_height_up,
                                         sh_x + sh_x2, sh_y1 + sh_y2)
            #rospy.logwarn('{},{}: {} {}'.format(pos, side, sh_x + sh_x2, sh_y1 + sh_y2))
            # add animation
            self.add_animation_description(pos, side, 'walk',
                                           limb.animations['walk'].moments,
                                           poses)

    def _generate_limbs(self, first_parts_pos, limb_parts_poses):
        self.limbs = {}
        # for all known limbs
        for (side, pos), (xyz, rpy) in first_parts_pos.items():
            # generate limb description:
            # [dx, dy, origin_dir, *lengths]
            coords = get_floats_from_string(xyz)
            if len(coords) < 2:
                rospy.logwarn('ant_motion_controller: processing of limb {}_{} skipped'
                              'due to wrong coordinates of first link'.format(side, pos))
                continue
            
            rot = get_floats_from_string(rpy)
            if len(rot) < 3:
                rospy.logwarn('ant_motion_controller: processing of limb {}_{} skipped'
                              'due to wrong orientation of first link'.format(side, pos))
                continue
            limb_descr = [coords[0], coords[1], rot[2]]
            # add length of each limb to description
            for p_str in limb_parts_poses[(side, pos)]:
                cs = get_floats_from_string(p_str)
                limb_descr.append(cs[0])
            self.limbs[(pos, side)] = LimbDescr(pos, side, 
                                                len(limb_parts_poses[(side, pos)])+1,
                                                limb_descr)
    
    def _load_description_from_xml_model(self, descr_str):
        # load data
        descr_xml = ET.fromstring(descr_str)
        # find all joints
        j_data = []
        for el in descr_xml:
            if el.tag == 'joint':
                parent = None
                child = None
                orig_xyz = None
                orig_rpy = None
                for j_el in el:
                    if j_el.tag == 'parent':
                        parent = j_el.attrib['link']
                    elif j_el.tag == 'child':
                        child = j_el.attrib['link']
                    elif j_el.tag == 'origin':
                        orig_xyz = j_el.attrib['xyz']
                        orig_rpy = j_el.attrib['rpy']
                j_data.append((parent, child, orig_xyz, orig_rpy))
        # calculate parameters for each limb:
        # get first parts for each limb
        first_parts_pos = {}
        limbs_first_part = []
        for parent, child, xyz, rpy in j_data:
            if parent == 'body_link':
                side,pos,order = get_limb_and_pos_from_str(child)
                if side is None:
                    continue
                else:
                    first_parts_pos[(side, pos)] = (xyz, rpy)
                    limbs_first_part.append(child)
        # get full data about each limb
        limb_parts_poses = {(sd,ps) : []
            for (sd, ps), (xyz, rpy) in first_parts_pos.items()}
        for fp in limbs_first_part:
            cfp = fp
            side,pos,order = get_limb_and_pos_from_str(fp)
            found = True
            while found:
                found = False
                for parent, child, xyz, rpy2 in j_data:
                    if parent == cfp:
                        cfp = child
                        limb_parts_poses[(side,pos)].append(xyz)
                        found = True
                        break
        return (first_parts_pos, limb_parts_poses)
        
        

    def add_animation_description(self, pos, side, name, moments, poses):
        self.limbs[(pos,side)].add_animation_description(name, moments, poses)

    def plan_animation_execution_loop(self, pos, side, name, start_moment):
        self.anim_plan[(pos, side)] = (name, start_moment, None)

    def plan_animation_execution_num(self, pos, side, name, start_moment, ntimes):
        self.anim_plan[(pos, side)] = (name, start_moment, ntimes)

    def prepare_for_animation(self):
        self.generate_get_up_animation()
        self.generate_walking_animation()

    def start_animations(self):
        self.animation_start_time = rospy.Time.now()
   
    def finish_animations(self):
        while self.anim_plan:
            rospy.sleep(0.1)

    def execute_animations(self, curr_time):
        if self.animation_start_time is not None:
            time_from_start = (curr_time - self.animation_start_time).to_sec()
            anims_to_stop = []
            for limb in self.limbs.keys():
                if limb in self.anim_plan.keys():
                    name, start_moment, number_of_times = self.anim_plan[limb]
                    # if number_of_times is None or animation is not yet finished, execute it
                    duration = self.limbs[limb].get_animation_duration(name)
                    if number_of_times is None or time_from_start < number_of_times*duration:
                        self.limbs[limb].execute_animation(name, start_moment, time_from_start)
                    else:
                        # stop animation
                        anims_to_stop.append(limb)
            # drop old animations
            for ats in anims_to_stop:
                del self.anim_plan[ats]

    def cb_j_st(self, msg):
        # update all joints (i.e. all limbs)
        for k in self.limbs.keys():
            self.limbs[k].update_current_pos(msg)
        
    def cb_anim_timer(self, e):
        curr_time = rospy.Time.now()
        # update animation commands
        self.execute_animations(curr_time)
        # execute commands
        for k in self.limbs.keys():
            self.limbs[k].process(curr_time)

    def generate_get_up_animation(self):
        """ getting up """
        time_moments = [0., 1., 2.]
        # for each limb generate motions
        for (pos,side), limb in self.limbs.items():
            # calculate disps
            dx = self.dx_zero[pos]
            dy = self.dy_zero[pos] + self.leg_y
            if side == 'right':
                dy = -dy
            poses = [limb.fk_local(0.,0.,0.,0)]
            # move legs to body
            #p1 = limb.ik_local(dx, dy, 0)
            p1 = (dx, dy, 0)
            #rospy.logerr('{},{}: {}'.format(pos, side, p1))
            #exit(-1)
            poses.append(p1)
            # move body up
            #p2 = limb.ik_local(dx, dy, -self.body_height_down)
            p2 = (dx, dy, -self.body_height_down)
            poses.append(p2)
            # add animation
            self.add_animation_description(pos, side, 'get_up',
                                           time_moments,
                                           poses)
    
    def generate_walking_animation(self):
        """ step movements """
        get_up_time = self.cycle_time * 0.25
        move_time = self.cycle_time * 0.5
        get_down_time = self.cycle_time * 0.25
        time_moments = [0.,
                        move_time/2, 
                        move_time/2 + get_down_time, 
                        move_time/2 + get_down_time + get_up_time,
                        3*move_time/2 + get_down_time + get_up_time,
                        3*move_time/2 + get_up_time + 2*get_down_time,
                        3*move_time/2 + 2*get_up_time + 2*get_down_time,
                        2*(move_time + get_up_time + get_down_time)]
        for (pos, side), limb in self.limbs.items():
            # calculate disps
            dx = self.dx_zero[pos]
            dy = self.dy_zero[pos] + self.leg_y
            if side == 'right':
                dy = -dy
            # get positions
            poses = gen_walking_template(dx, dy,
                                         self.body_height_down, self.body_height_up,
                                         0.3, 0.)
            # add animation
            self.add_animation_description(pos, side, 'walk',
                                           time_moments,
                                           poses)
            
    
    def get_up(self):
        self.plan_animation_execution_num('forward', 'left', 'get_up', 0., 1)
        self.plan_animation_execution_num('middle', 'left', 'get_up', 0., 1)
        self.plan_animation_execution_num('backward', 'left', 'get_up', 0., 1)
        self.plan_animation_execution_num('forward', 'right', 'get_up', 0., 1)
        self.plan_animation_execution_num('middle', 'right', 'get_up', 0., 1)
        self.plan_animation_execution_num('backward', 'right', 'get_up', 0., 1)
        # process movements
        self.start_animations()
        self.finish_animations()
        
    def walk(self):
        self.plan_animation_execution_loop('forward', 'left', 'walk', 0.)
        self.plan_animation_execution_loop('middle', 'left', 'walk', self.cycle_time)
        self.plan_animation_execution_loop('backward', 'left', 'walk', 0.)
        self.plan_animation_execution_loop('forward', 'right', 'walk', self.cycle_time)
        self.plan_animation_execution_loop('middle', 'right', 'walk', 0.)
        self.plan_animation_execution_loop('backward', 'right', 'walk', self.cycle_time)
        self.start_animations()
        
    def walk_diagonal(self):
        self.plan_animation_execution_loop('forward', 'left', 'walk', 0.)
        self.plan_animation_execution_loop('middle', 'left', 'walk', self.cycle_time+1)
        self.plan_animation_execution_loop('backward', 'left', 'walk', self.cycle_time)
        self.plan_animation_execution_loop('forward', 'right', 'walk', self.cycle_time)
        self.plan_animation_execution_loop('middle', 'right', 'walk', self.cycle_time+1)
        self.plan_animation_execution_loop('backward', 'right', 'walk', 0.)
        self.start_animations()
        
    def walk_one_by_one(self):
        self.plan_animation_execution_loop('forward', 'left', 'walk', self.cycle_time)
        self.plan_animation_execution_loop('middle', 'left', 'walk', self.cycle_time+1)
        self.plan_animation_execution_loop('backward', 'left', 'walk', self.cycle_time+2)
        self.plan_animation_execution_loop('forward', 'right', 'walk', self.cycle_time+3)
        self.plan_animation_execution_loop('middle', 'right', 'walk', self.cycle_time+4)
        self.plan_animation_execution_loop('backward', 'right', 'walk', self.cycle_time+5)
        self.start_animations()
        
    def walk_on_four(self):
        #self.plan_animation_execution_loop('forward', 'left', 'walk', self.cycle_time)
        self.plan_animation_execution_loop('middle', 'left', 'walk', 0.)
        self.plan_animation_execution_loop('backward', 'left', 'walk', self.cycle_time)
        #self.plan_animation_execution_loop('forward', 'right', 'walk', self.cycle_time+3)
        self.plan_animation_execution_loop('middle', 'right', 'walk', self.cycle_time)
        self.plan_animation_execution_loop('backward', 'right', 'walk', 0.)
        
if __name__ == '__main__':
    try:
        rospy.init_node('ant_motion_controller', anonymous=True)
        # ant description
        ant_descr = rospy.get_param('robot_description')
        
        step_dist = 0.7
        cycle_time = 1
        ad = AntDescr(ant_descr, step_dist, cycle_time)
        
        ad.prepare_for_animation() # generate poses
        ad.get_up()
        #ad.walk()
        #ad.walk_diagonal()
        ad.walk_one_by_one()
        #ad.walk_on_four()
        
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
        
