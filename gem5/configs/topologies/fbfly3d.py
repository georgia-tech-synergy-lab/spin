# Copyright (c) 2016 Georgia Institute of Technology
# All rights reserved.
#
# Authors: Aniruddh Ramrakhyani
#          

from m5.params import *
from m5.objects import *

from BaseTopology import SimpleTopology

#create a 3-D flattened butterfly topology
#all link have equal weights (this can be overriden
#when defining the links.)
#Assume equal no of cache and directory controllers

class fbfly3d(SimpleTopology):
    description='3-D flattened butterfly'

    print "Creating Topology: " + description

    def __init__(self, controllers):
        self.nodes = controllers

    # Makes a 3-d flattened butterfly
    # assuming an equal number of cache and directory cntrls

    def makeTopology(self, options, network, IntLink, ExtLink, Router):
        nodes = self.nodes

        num_routers = options.num_cpus
        num_rows = options.fbfly_rows
        num_cols = options.fbfly_cols

        # default values for link latency and router latency.
        # Can be over-ridden on a per link/router basis
        link_latency = options.link_latency # used by simple and garnet
        router_latency = options.router_latency # only used by garnet

        # There must be an evenly divisible number of cntrls to routers
        # Also, obviously the number of rows must be <= the number of routers
        cntrls_per_router, remainder = divmod(len(nodes), num_routers)
        assert(num_rows > 0 and num_rows <= num_routers)
        assert(num_cols > 0 and num_cols <= num_routers)
        num_meshes = int(num_routers / (num_rows*num_cols))
        assert(num_meshes * num_rows * num_cols == num_routers)

        # Create the routers 
        routers = [Router(router_id=i, latency = router_latency) \
            for i in range(num_routers)]
        network.routers = routers

        # link counter to set unique link ids
        link_count = 0

        # Add all but the remainder nodes to the list of nodes to be uniformly
        # distributed across the network.
        network_nodes = []
        remainder_nodes = []
        for node_index in xrange(len(nodes)):
            if node_index < (len(nodes) - remainder):
                network_nodes.append(nodes[node_index])
            else:
                remainder_nodes.append(nodes[node_index])

        # Connect each node to the appropriate router
        ext_links = []
        for (i, n) in enumerate(network_nodes):
            cntrl_level, router_id = divmod(i, num_routers)
            assert(cntrl_level < cntrls_per_router)
            ext_links.append(ExtLink(link_id=link_count, ext_node=n,
                                    int_node=routers[router_id],
                                    latency = link_latency))
            link_count += 1

        # Connect the remainding nodes to router 0.  These should only be
        # DMA nodes.
        for (i, node) in enumerate(remainder_nodes):
            assert(node.type == 'DMA_Controller')
            assert(i < remainder)
            ext_links.append(ExtLink(link_id=link_count, ext_node=node,
                                    int_node=routers[0],
                                    latency = link_latency))
            link_count += 1

        network.ext_links = ext_links

        # Create the 3-D fbfly links.
        int_links = []

        print "East outport -- West Inport links "

        # East output to West input links (weight = 1)
        for mesh in xrange(num_meshes):
            for row in xrange(num_rows):
                for col in xrange(num_cols):
                    if (col + 1 < num_cols):
                        east_out = (mesh*num_rows*num_cols) + col + (row * num_cols)
                        for west_in in range(east_out+1, (mesh*num_rows*num_cols) + (row * num_cols) + num_cols):
                            print "Router " + get_id(routers[east_out]) + " created a link to Router " +  get_id(routers[west_in])
                            int_links.append(IntLink(link_id=link_count,
                                                     src_node=routers[east_out],
                                                     dst_node=routers[west_in],
                                                     src_outport="East",
                                                     dst_inport="West",
                                                     latency = link_latency,
                                                     weight=1))
                            link_count += 1



        print "West outport -- East Inport links "

        # West output to East input links (weight = 1)
        for mesh in xrange(num_meshes):
            for row in xrange(num_rows):
                for col in xrange(num_cols):
                    if (col + 1 < num_cols):
                        west_out = (mesh*num_rows*num_cols) + (col + 1) + (row * num_cols)
                        for east_in in range(west_out-1, (mesh*num_rows*num_cols) + (row * num_cols) - 1, -1):     
                            print "Router " + get_id(routers[west_out]) + " created a link to Router " +  get_id(routers[east_in])
                            int_links.append(IntLink(link_id=link_count,
                                                     src_node=routers[west_out],
                                                     dst_node=routers[east_in],
                                                     src_outport="West",
                                                     dst_inport="East",
                                                     latency = link_latency,
                                                     weight=1))
                            link_count += 1



        print "North outport -- South Inport links "

        # North output to South input links (weight = 1)
        for mesh in xrange(num_meshes):
            for row in xrange(num_rows):
                if (row + 1 < num_rows):
                    for col in xrange(num_cols):
                        north_out = (mesh*num_rows*num_cols) + (row * num_cols) + col 
                        for south_in in range(north_out + num_cols, (mesh*num_rows*num_cols) + ((num_rows-1) * num_cols) + col + 1, 
                                              num_cols):
                            print "Router " + get_id(routers[north_out]) + " created a link to Router " +  get_id(routers[south_in])
                            int_links.append(IntLink(link_id=link_count,
                                                     src_node=routers[north_out],
                                                     dst_node=routers[south_in],
                                                     src_outport="North",
                                                     dst_inport="South",
                                                     latency = link_latency,
                                                     weight=1))
                            link_count += 1



        print "South outport -- North Inport links "

        # South output to North input links (weight = 1)
        for mesh in xrange(num_meshes):
            for row in xrange(num_rows):
                if (row + 1 < num_rows):
                    for col in xrange(num_cols):
                        south_out = (mesh*num_rows*num_cols) + ((row + 1) * num_cols) + col
                        for north_in in range(south_out - num_cols, (mesh*num_rows*num_cols) + col - 1, -num_cols): 
                            print "Router " + get_id(routers[south_out]) + " created a link to Router " +  get_id(routers[north_in])
                            int_links.append(IntLink(link_id=link_count,
                                                     src_node=routers[south_out],
                                                     dst_node=routers[north_in],
                                                     src_outport="South",
                                                     dst_inport="North",
                                                     latency = link_latency,
                                                     weight=1))
                            link_count += 1



        print "Up outport -- Down Inport links "

        # Up output to Down input links (weight = 1)
        for mesh in xrange(num_meshes):
            if (mesh + 1 < num_meshes):
                for row in xrange(num_rows):
                    for col in xrange(num_cols):
                        up_out = (mesh*num_rows*num_cols) + (row * num_cols) + col 
                        for down_in in range(up_out + (num_rows*num_cols), 
                                             ((num_meshes-1)*num_rows*num_cols) + (row * num_cols) + col + 1, num_rows*num_cols):
                            print "Router " + get_id(routers[up_out]) + " created a link to Router " +  get_id(routers[down_in])
                            int_links.append(IntLink(link_id=link_count,
                                                     src_node=routers[up_out],
                                                     dst_node=routers[down_in],
                                                     src_outport="Up",
                                                     dst_inport="Down",
                                                     latency = link_latency,
                                                     weight=1))
                            link_count += 1


        print "Down outport -- Up Inport links "

        # Down output to Up input links (weight = 1)
        for mesh in xrange(num_meshes):
            if (mesh + 1 < num_meshes):
                for row in xrange(num_rows):
                    for col in xrange(num_cols):
                        down_out = ((mesh+1)*num_rows*num_cols) + (row * num_cols) + col
                        for up_in in range(down_out - (num_rows*num_cols),  (row * num_cols) + col - 1, -(num_rows*num_cols)): 
                            print "Router " + get_id(routers[down_out]) + " created a link to Router " +  get_id(routers[up_in])
                            int_links.append(IntLink(link_id=link_count,
                                                     src_node=routers[down_out],
                                                     dst_node=routers[up_in],
                                                     src_outport="Down",
                                                     dst_inport="Up",
                                                     latency = link_latency,
                                                     weight=1))
                            link_count += 1

        network.int_links = int_links

def get_id(node) :
    return str(node).split('.')[3].split('routers')[1]
