import swarm

def myload(me):
    print('myload: %s'%(me))
    swarm.bind(me, me, 1234)
  
def myupdate(me, world_name, sim_time, real_time):
    print('myupdate: %s %s %f %f'%(me, world_name, sim_time, real_time))
    swarm.send_to(me, "foo", "broadcast", 1234)

def myondatareceived(me, src_address, dst_address, dst_port, data):
    print('myondatareceived: %s %s %s %d %s'
          %(me, src_address, dst_address, dst_port, data))

