import swarm

def myload(me):
    swarm.gzmsg('[%s] load()'%(me))
    swarm.gzmsg('[%s] vehicle type: %s'%(me, swarm.type(me)))
    search_area = swarm.search_area(me)
    swarm.gzmsg('[%s] search area: [%f, %f, %f, %f]' % 
      (me, search_area[0], search_area[1], search_area[2], search_area[3]))
    lost_person_dir = swarm.lost_person_dir(me)
    swarm.gzmsg('[%s] lost person dir: [%f, %f]' % 
      (me, lost_person_dir[0], lost_person_dir[1]))
    boo_pose = swarm.boo_pose(me)
    swarm.gzmsg('[%s] boo pose: [%f, %f]' % 
      (me, boo_pose[0], boo_pose[1]))

    swarm.bind(me, swarm.host(me), 1234)
    swarm.bind(me, "multicast", 1234)
  
def myupdate(me, world_name, sim_time, real_time):
    swarm.gzmsg('[%s] update()'%(me))
    for n in swarm.neighbors(me):
      swarm.send_to(me, "unicast to %s"%(n), n, 1234)
    swarm.send_to(me, "broadcast to everyone", "broadcast", 1234)
    swarm.send_to(me, "multicast to some", "multicast", 1234)

    swarm.set_linear_velocity(me, 0.5, 0, 0)
    swarm.set_angular_velocity(me, 0, 0, 0.25)
    pose = swarm.pose(me)

    swarm.gzmsg('[%s] pose: [%f, %f, %f]'%(me, pose[0], pose[1], pose[2]))

    imu = swarm.imu(me)
    swarm.gzmsg('[%s] imu: [%f, %f, %f, %f, %f, %f, %f, %f, %f]' %
      (me, imu[0], imu[1], imu[2],
       imu[3], imu[4], imu[5],
       imu[6], imu[7], imu[8]))

    bearing = swarm.bearing(me)
    swarm.gzmsg('[%s] bearing: %f'%(me, bearing))

    image = swarm.image(me)
    swarm.gzmsg('[%s] image: %s'%(me, image))

    swarm.gzmsg('[%s] terrain type: %s'%(me, swarm.terrain_type(me)))

    swarm.gzmsg('[%s] docked?: %d'%(me, swarm.is_docked(me)))

    swarm.launch(me)
    swarm.dock(me, me)

def myondatareceived(me, src_address, dst_address, dst_port, data):
    swarm.gzmsg('[%s] ondatareceived(): %s %s %d %s'
          %(me, src_address, dst_address, dst_port, data))

