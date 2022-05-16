#!/usr/bin/env python3
import math
import xml.etree.ElementTree as ET
import rospy
import re
re_limb_and_pos = re.compile('([a-zA-Z]+)_([a-zA-Z]+)([0-9]+)_link')
from std_msgs.msg import Float64


def gen_walking_template(dx, dy, height_down, height_up,
                         step_dist, step_dy):
    p1 = (dx, dy, -height_down)
    p2 = (dx - step_dist*0.5, dy - step_dy*0.5, -height_down)
    p3 = (dx - step_dist*0.5, dy - step_dy*0.5, -height_up)
    p4 = (dx + step_dist*0.5, dy + step_dy*0.5, -height_up)
    p5 = (dx + step_dist*0.5, dy + step_dy*0.5, -height_down)
    return [p1, p2, p2, p3, p4, p5, p5, p1]

# class for a single joint

class JointDescr(object):
    def __init__( self, pos, side, num ):
        self.pos = pos
        self.side = side
        self.num = num     
        self.pub = rospy.Publisher(self.get_cmd_topic_name(), Float64, queue_size=10)
            

    def get_cmd_topic_name(self):
        return '{}_{}{}_position_controller/command'.format( self.side,self.pos,self.num) 

    

    def set_pos(self, pos):

        msg = Float64()

        msg.data = pos

        self.pub.publish(msg)

# generate all joints of an ant
#def generate_joints():
 #   possible_pos def gen_walking_template(dx, dy, height_down, height_up,
                         step_dist, step_dy):
    p1 = (dx, dy, -height_down)
    p2 = (dx - step_dist*0.5, dy - step_dy*0.5, -height_down)
    p3 = (dx - step_dist*0.5, dy - step_dy*0.5, -height_up)
    p4 = (dx + step_dist*0.5, dy + step_dy*0.5, -height_up)
    p5 = (dx + step_dist*0.5, dy + step_dy*0.5, -height_down)
    return [p1, p2, p2, p3, p4, p5, p5, p1]= ['forward', 'middle', 'backward']
 #   possible_sides = ['left', 'right']
 #   num = 4
 #   joints = []
 #   for p in possible_pos:
 #       for s in possible_sides:
 #           for i in range(num):
 #               joints.append(JointDescr(p,s,i))
 #   return joints


def get_limb_and_pos_from_str(limb_pos_str):
    m = re_limb_and_pos.match(limb_pos_str)
    if m is None:
        rospy.logwarn('ant_motion_controller: unknown limb and pos of link {}'.format(limb_pos_str))
        return (None, None, None)
    else:
        return (m.group(1), m.group(2), int(m.group(3)))  



# creating single leg by using joints generator 

class SingleLeg(object):
    def __init__(self, side, pos, num):
        self.pos = pos
        self.side = side
        self.num = num #number of joints
        self.joints = [JointDescr(pos, side, i)
          for i in range(1,num+1)]


#creating Ant using six leg generators

class FullAnt(object):
    def __init__(self):

        possible_pos = ['forward', 'middle', 'backward']
        possible_sides = ['left', 'right']
        self.legs = {}
        for p in possible_pos:
            for s in possible_sides:
                self.legs[(p,s)] = SingleLeg(s,p, 4)

                        

        #self.pos = pos

        #self.side = side

        #self.num = num

        

        

    def load_description_from_xml_model(self, descr_str):

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

class InversedKinematics(object): 
    def __init__(self, leg_name, fa, goal_pos):

        leg_descr = []
        leg_init = []
        global leg_part_1
        global leg_part_2 
        global leg_part_3       
        leg_init, leg_descr = fa.load_description_from_xml_model(rospy.get_param('robot_description'))

        #rospy.logerr(list(leg_descr.keys()))
        for keys in leg_descr.keys():
            if keys == leg_name:
                leg_data = leg_descr[keys]
                #rospy.logerr("LD: {}".format(leg_data))
                lengths = [float(ld.split()[0]) for ld in leg_data]
                #rospy.logerr('Ls: {}'.format(lengths))
        for keys in leg_init.keys():
            if keys == leg_name:
                leg_pos = leg_init[keys]
                #rospy.logerr("LD: {}".format(leg_data))
                poses = [[float(y) for y in lp.split()] for lp in leg_pos]        
        leg_part_1 = lengths[0]
        leg_part_2 = lengths[1]
        leg_part_3 = lengths[2]
        leg_pos_1 =  poses
        rospy.logerr("'1 - {0}','2 - {1}', '3 - {2}'".format (leg_part_1,leg_part_2,leg_part_3))
        rospy.logerr('core - {0}'.format (leg_pos_1))      
    
                    

    def angle_base_1 (goal_pos):
        l1 = leg_part_1  # длина первого элемента ноги
        rospy.logerr('l1 - {0}'.format (l1))
        l2 = math.sqrt((goal_pos[0] - leg_part_1)**2 + (goal_pos[1] - 0)**2) 
        rospy.logerr('l2 - {0}'.format (l2))
        l3 = math.sqrt((goal_pos[0] - 0)**2 + (goal_pos[1]-0)**2)
        rospy.logerr('l3 {0}'.format (l3))
        sigma = (math.acos((l1**2 + l2**2 - l3**2)/(2*l2*l1)))* 180/3,14
        rospy.logerr('angle s- {0}'.format (sigma))
        return sigma

    

    def angle_1_2 (goal_pos):
        beta = (math.atan2(leg_part_1, leg_part_2))*180/3,14
        rospy.logerr('angle b - {0}'.format(beta))
        return beta


    def angle_2_3 (goal_pos):
        l1 = math.sqrt((goal_pos[1] - 0)**2 +(goal_pos[2] - 0)) #чет не то, надо переписать
        rospy.logerr('l1 - {0}'.format(l1))
        l2 = leg_part_2
        rospy.logerr('l2 - {0}'.format(l2))
        l3 = leg_part_3
        rospy.logerr('l3 - {0}'.format(l3))
        a = ((l3**2 + l2**2 - l1**2)/(2*l2*l3))
        rospy.logerr('a - {0}'.format(a))
        alpha = (math.acos(0.001*((l3**2 + l2**2 - l1**2)/(2*l2*l3)))) * 180/3,14
        rospy.logerr('angle a - {0}'.format(alpha))
        return alpha

   

class PoseController(object):

    def __init__(self, side, pos, num):

        tripod = [[0, 1, 1, 0, 0, 1],
                  [1, 0, 0, 1, 1, 0]]

        bipod = [[1, 1, 0, 0, 0, 0],
                 [0, 0, 1, 1, 0, 0],
                 [0, 0, 0, 0, 1, 1]]

        careful = [[1, 0, 0, 0, 0, 0],
                   [0, 1, 0, 0, 0, 0],
                   [0, 0, 1, 0, 0, 0],
                   [0, 0, 0, 1, 0, 0],
                   [0, 0, 0, 0, 1, 0],
                   [0, 0, 0, 0, 0, 1]]

                   

        # добавить походок 

        # 0 - отсутсвие движения

        # 1 - нога поднята, поворот не осуществляется

        # 2 - поворот ноги, углы не меняются

        # 3 - опускание ноги

        # 4 - поворот стоя на земле

        single_leg_matrix = [[0, 1, 2, 3, 4]]
        
        
    def changer (walking_cycle, gait):
        while walking_cycle == 1:
            for i in len(gait):
                if gait[j][i] == 1:
                    poser()
                if gait[j][i] == 0:
                    pass
        if walking_cycle == 0:
            rospy.logerr('walking_cycle is over')
    
       
        
        
        
        
        

    def poser():

        single_leg_matrix = [[0, 1, 2, 3, 4]]

        for i in len(single_leg_matrix):

            if single_leg_matrix[j][i] == 0: 
                pass

            if single_leg_matrix[j][i] == 1:
                a = InversedKinematics.angle_1_2(goal_pos)
                rospy.logerr('1 - {0}'.format(a))
            if single_leg_matrix[j][i] == 2:
                b = InversedKinematics.angle_base_1(goal_pos)
                rospy.logerr('2 - {0}'.format(b))
            if single_leg_matrix[j][i] == 3:
                c = InversedKinematics.angle_1_2(goal_pos)
                rospy.logerr('3 - {0}'.format(c))
                d = InversedKinematics.angle_2_3(goal_pos)
                rospy.logerr('3 - {0}'.format(d))
            if single_leg_matrix[j][i] == 4:
                e = InversedKinematics.angle_base_1(goal_pos)
                rospy.logerr('4 - {0}'.format(e))

class joint_publisher(goal_pos):
    pub = rospy.Publisher(InversedKinematics(('left', 'forward')))
    rospy.init_node ('IK')


if __name__ == '__main__':

    try:

        rospy.init_node('ant_controller')
        fa = FullAnt()
        gp = [0.1, 0.03, 0.04]

        #p1,p2 = fa.load_description_from_xml_model(rospy.get_param('robot_description'))

        #rospy.logerr('{}'.format( p2))

        lf = InversedKinematics(('left', 'forward'), fa, gp)
        ss = InversedKinematics.angle_base_1(gp)
        sb = InversedKinematics.angle_1_2(gp)
        sa = InversedKinematics.angle_2_3(gp)

        #rb = InversedKinematics(('right', 'backward'), fa)

        rospy.spin()

    except rospy.ROSInterruptException:

        pass

