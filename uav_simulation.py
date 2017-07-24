
from pymavlink import mavutil
from threading import Thread
# from Queue import Queue
import time


from uav_master import uav_master
from uav_slave import uav_slave
from multiprocessing import Queue
from uav_pre_plot import wp_sphere_plot


def initialize_vehicles():

    '''todo master obj api: requires angle of evelvation() to define number of turns,
     groundspeed, radius, threadsafe Queue, master-slave NE dist, alt offset, '''
    #todo call swarm plot module func. with 100 point to show the plot, with already defined
    #todo ..radius, groundspeed etc
    shared_q = Queue(10)
    shared_q_1 = Queue(10)

    hm_loc_q = Queue(1)
    hm_loc_q_1 = Queue(1)



    print "Simulation Start wait for 7 sec.."

    radius = 20
    master_groundspeed = 1.7
    initial_alt = 10
    turns = 10

    master_slave_n_e = 5
    master_slave_d = 5
    slave_groundspeed = 5

    master_slave_n_e_1 = -5
    master_slave_d_1 = -5
    slave_groundspeed_1 = 5

    wp_sphere_plot(turns, radius,initial_alt, master_groundspeed,master_slave_n_e, master_slave_d)

    time.sleep(7)


    copter_master = uav_master('127.0.0.1:14553', initial_alt, radius,
                               turns, master_groundspeed)

    # copter_master = uav_master('127.0.0.1:14553', initial_alt, radius,
    #                            turns, master_groundspeed, shared_q=shared_q, hm_loc_q=hm_loc_q,
    #                            shared_q_1=shared_q_1, hm_loc_q_1=hm_loc_q_1)

    # copter_master = uav_master('127.0.0.1:14553', initial_alt, radius,
    #                            turns, master_groundspeed, shared_q=shared_q, hm_loc_q=hm_loc_q)
    #
    #
    #
    # copter_slave = uav_slave('127.0.0.1:14554', initial_alt, shared_q,
    #                          slave_groundspeed,master_slave_n_e, master_slave_d, hm_loc_q)
    #
    # copter_slave_1 = uav_slave('127.0.0.1:14557', initial_alt, shared_q_1,
    #                            slave_groundspeed_1, master_slave_n_e_1, master_slave_d_1, hm_loc_q_1)

    copter_master.start()

    # copter_slave.start()
    #
    # copter_slave_1.start()


    print "master joined"
    copter_master.join()
    # print "slave joined"
    # copter_slave.join()
    # print "slave 1 joined"
    # copter_slave_1.join()



if __name__ == '__main__':
    initialize_vehicles()
