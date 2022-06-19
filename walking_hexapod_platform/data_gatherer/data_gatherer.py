#!/usr/bin/env python3

import rospy

from gazebo_msgs.msg import ModelStates

time_start = None
dist_start = 1
time_end = None
dist_end = 2



out_file = None
step_dist = None
cycle_time = None

def cb_model_states(msg):
    global time_start
    global time_end
    global dist_start
    global dist_end
    # rospy.logerr ('smth')
    # get index of the ant
    if 'walking_hexapod_platform' not in msg.name:
        return
    ant_index = msg.name.index('walking_hexapod_platform')
    pos = msg.pose[ant_index].position
    dist = (pos.x ** 2 + pos.y ** 2) ** 0.5
    rospy.logerr('x {:.5f}, y {:.5f}'.format(pos.x, pos.y))
    rospy.logerr('dist: {:.5f}'.format(dist))
    curr_time = rospy.Time.now()
    if dist > dist_start and time_start is None:
        time_start = curr_time
    
    if dist > dist_end and time_end is None:
        time_end = curr_time
        # generate output
        exec_time = ((time_end - time_start).to_sec())
        if exec_time < 1e-4:
            rospy.logerr('model failed')
        else:
            # write result to output file
            with open(out_file, 'a') as f:
                f.write('{:.5f}; {:.5f};{:.5f}; {:.5f}; \n'.format(step_dist, 2*cycle_time, exec_time, (dist_end-dist_start)/exec_time))
        # exit;step_period;movement_time;velocity
        rospy.signal_shutdown('gathering finished')
        

if __name__ == '__main__':
    rospy.init_node('data_gatherer')
    out_file = rospy.get_param('~out_file')
    step_dist = rospy.get_param('~step_dist')
    cycle_time = rospy.get_param('~cycle_time')
    s = rospy.Subscriber('/gazebo/model_states', ModelStates, cb_model_states)
    rospy.spin()
    
    
