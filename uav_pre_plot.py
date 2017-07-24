from mpl_toolkits.mplot3d import axes3d
import matplotlib.pyplot as plt
import numpy as np
import math
from math import sqrt, cos, sin, asin, atan2, tan

inst_speed = lambda elev, uav_tocenter, telem_rate: (cos(elev) * uav_tocenter * 2 * math.pi * telem_rate) / 360

def wp_sphere_plot(turns, radius, ini_height, groundspeed, mst_slv_dist, alt_off):
    #todo get elevation angle or turns, groundspeed, init_height, master-slave dist, alt-off

    fig = plt.figure()
    ax = fig.gca(projection='3d')

    # turns = 10
    # radius = 20
    # ini_height = 10
    total_points = 100 * turns
    phi = 2 * math.pi / 100
    theta = phi / 4 / turns

    n_prev = 0
    e_prev = 0
    d_prev = 0
    total_dist = 0
    total_dist_t = 0

    first_iter = True
    magvec_set = False
    ini_dist_off = 0

    # todo w/o az_res check the inst speed
    az = 1.2 #esitmation on positioning points
    inst_speed = lambda elev, uav_mag, telem_rate: (cos(elev) * uav_mag * 2 * math.pi * az *telem_rate) / 360
    time_travel = 0.0

    # # todo simulate slave pos
    # mst_slv_dist = 5
    # alt_off = 5

    # groundspeed = 1.7

    s_d = alt_off

    nlis = []
    elis = []
    dlis = []
    unlist = []
    uelist = []
    udlist = []

    #todo if roi is used
    # roi_bearing = 0
    # n_roi = 10 * cos(math.radians(roi_bearing))
    # e_roi = 10 * sin(math.radians(roi_bearing))
    # d_roi = -ini_height
    # total_points = total_points + 1
    for pnt in range(total_points):
        north = radius * math.cos(theta * pnt) * math.cos(phi * pnt)
        nlis.append(north)
        east = radius * math.cos(theta * pnt) * math.sin(phi * pnt)
        elis.append(east)
        alt = ini_height * (radius * math.sin(theta * pnt))
        dlis.append(alt)

        if (first_iter == True):
            n_curr = north
            e_curr = east
            d_curr = alt
            first_iter = False
            est_magvec_abs = (sqrt(pow(n_curr, 2) + pow(e_curr, 2) + pow(d_curr, 2)))
            est_magvec_relative = (sqrt(pow(n_curr, 2) + pow(e_curr, 2) + pow(d_curr, 2))) - 10
            # est_mag_roi = (sqrt(pow(n_roi - n_curr, 2) + pow(e_roi - e_curr, 2) + pow(d_roi - d_curr, 2)))

            sph_alt = abs(d_curr) - ini_height
            sph_ang = asin((sph_alt / est_magvec_relative))
            gd_center = sph_alt / tan(sph_ang)

            Un = (n_curr / est_magvec_relative)
            Ue = (e_curr / est_magvec_relative)
            Ud = (d_curr / est_magvec_relative)

            #absolute scaling
            # SUn = Un * (abs(alt_off))
            # SUe = Ue * (abs(alt_off))
            # SUd = Ud * (abs(alt_off))

            est_mst_slv_dist_n = gd_center + mst_slv_dist * cos(math.radians(90))
            est_mst_slv_dist_e = gd_center + mst_slv_dist * sin(math.radians(90))

            SUn = Un * (est_mst_slv_dist_n )
            SUe = Ue * (est_mst_slv_dist_e )
            SUd = Ud * (abs(alt_off))
            if(alt_off > 0):

            # est_mst_slv_dist = gd_center + mst_slv_dist
                slave_n = n_curr + (SUn)  # put slave n and e coord for assesment
                slave_e = e_curr + (SUe)
                slave_d = d_curr + SUd
            else:
                slave_n = n_curr - (SUn)  # put slave n and e coord for assesment
                slave_e = e_curr - (SUe)
                slave_d = d_curr - SUd

            uelist.append(slave_e)
            unlist.append(slave_n)
            udlist.append(slave_d)


        else:
            n_curr = north
            e_curr = east
            d_curr = alt

            # euclidean dist/direct dist
            pnt_dist = (sqrt(pow(n_curr - n_prev, 2) + pow(e_curr - e_prev, 2) + pow(d_curr - d_prev, 2)))
            print "\npoint distance %f" % (pnt_dist)

            est_magvec_abs = (sqrt(pow(n_curr, 2) + pow(e_curr, 2) + pow(d_curr, 2)))
            print "Estimated direct distance magnitude: %f" % (est_magvec_abs)
            if(magvec_set == False):
                ini_dist_off = est_magvec_abs
                magvec_set = True

            est_magvec_relative = (sqrt(pow(n_curr, 2) + pow(e_curr, 2) + pow(d_curr, 2))) - 10
            print "Estimated magnitude vector relative: %f" % (est_magvec_relative)

            # est_mag_roi = (sqrt(pow(n_roi - n_curr, 2) + pow(e_roi - e_curr, 2) + pow(d_roi - d_curr, 2)))

            cent_theta = math.acos(1 - (pow(pnt_dist, 2) / (2 * (pow(radius, 2)))))
            total_dist_t = radius * cent_theta


            # unit vectors testing
            Un = (n_curr / est_magvec_abs)
            Ue = (e_curr / est_magvec_abs)
            Ud = (d_curr / est_magvec_abs)
            # SUn = Un * (abs(alt_off))
            # SUe = Ue * (abs(alt_off))
            # SUd = Ud * (abs(alt_off))

            sph_alt = abs(d_curr) - ini_height
            sph_ang = asin((sph_alt / est_magvec_relative))
            gd_center = sph_alt / tan(sph_ang)
            print "GROUND DIST TO THE CENTER = {0}".format(gd_center)
            ele_ang = math.degrees(atan2(sph_alt, gd_center)) #verifiy results with WGS approach
            print "ELEVATION ANGLE = {0}".format(ele_ang)

            est_mst_slv_dist_n = gd_center + mst_slv_dist * math.radians(cos(90))
            est_mst_slv_dist_e = gd_center + mst_slv_dist * math.radians(sin(90))
            SUn = Un * (est_mst_slv_dist_n)
            SUe = Ue * (est_mst_slv_dist_e)
            SUd = Ud * (abs(alt_off))

            if(alt_off > 0):
                slave_n = n_curr + (SUn)
                slave_e = e_curr + (SUe)
                slave_d = d_curr + SUd
            else:
                slave_n = n_curr - (SUn)
                slave_e = e_curr - (SUe)
                slave_d = d_curr - SUd
            print "north: %f east: %f down: %f" % (n_curr, e_curr, d_curr)
            print "slave_n: %f slave_e: %f slave_d: %f" % (slave_n, slave_e, slave_d)
            print "Unit Vectors:: n: %f e: %f d: %f" % (Un, Ue, Ud)
            print "Scaled U:: n: %f e: %f d: %f" % (SUn, SUe, SUd)
            u_mag = (sqrt(pow(Un, 2) + pow(Ue, 2) + pow(Ud, 2)))
            print "magnitude unit vector: %f" % (u_mag)


            hyp_dist = (gd_center / math.cos(math.radians(ele_ang))) #should be equal magvec_relative
            print "HYP DIST:{0}".format(hyp_dist)
            maxspeed = inst_speed(math.radians(ele_ang), est_magvec_relative, 10)
            if (groundspeed < maxspeed):
                print "GS {0}".format(groundspeed)
                time_travel = time_travel + (total_dist_t / groundspeed)
            else:
                print "MAXSPEED: {0}".format(maxspeed)
                time_travel = time_travel + (total_dist_t / maxspeed)

            uelist.append(slave_e)
            unlist.append(slave_n)
            udlist.append(slave_d)

        total_dist = total_dist + total_dist_t
        n_prev = n_curr
        e_prev = e_curr
        d_prev = d_curr

    #todo adding the initial dist. uav need to travel to reach the start point on the sphere
    print "TOTAL DIST. ON A SPHERE: {0}".format(total_dist + ini_dist_off)
    print "AVERAGE TIME ON A SPHERE: {0}".format(time_travel)
    print "estimated AVERAGE SPEED on a SPHERE : %f m/s " % ((total_dist + ini_dist_off) / time_travel)
    ax.plot(elis, nlis, dlis, 'g-')
    ax.plot(uelist, unlist, udlist, 'r-' )

    # plt.draw()
    plt.show()

if __name__ == '__main__':
    wp_sphere_plot()