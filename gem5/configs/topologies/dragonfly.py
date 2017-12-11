# Copyright (c) 2016 Georgia Institute of Technology
# All rights reserved.
#
# Authors: Aniruddh Ramrakhyani
#          

from m5.params import *
from m5.objects import *

import math

from BaseTopology import SimpleTopology

# This file creates a dragon-fly topology.
# Both intra-group and inter-group interconnection networks
# are 1-D flattened buttefly or a fully connected topology.
#
# Assuming an equal number of cache
# and directory controllers.
# All links have equal weights.

class dragonfly(SimpleTopology):
    description='Dragon-fly topology'

    print "Creating Topology: " + description

    def __init__(self, controllers):
        self.nodes = controllers

    # Makes a dragon-fly topology with 1-d flattened butterfly toplogy for both 
    # inter-group and intra-group interconnection networks.
    # assuming an equal number of cache and directory cntrls

    def makeTopology(self, options, network, IntLink, ExtLink, Router):
        nodes = self.nodes

        num_routers = options.num_cpus
        group_size = options.dfly_group_size

        # default values for link latency and router latency.
        # Can be over-ridden on a per link/router basis
        link_latency = options.link_latency # used by simple and garnet
        router_latency = options.router_latency # only used by garnet

        num_groups = int(math.ceil(num_routers/group_size))

        global_links_per_router = int(math.ceil((num_groups)/group_size))


        # There must be an evenly divisible number of cntrls to routers
        # Also, obviously the group_size must be <= the number of routers
        cntrls_per_router, remainder = divmod(len(nodes), num_routers)
        assert(group_size > 0 and group_size <= num_routers)
        
        assert(num_groups * group_size == num_routers)

        # Create the routers in the mesh
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
            #assert(node.type == 'DMA_Controller')
            assert(i < remainder)
            ext_links.append(ExtLink(link_id=link_count, ext_node=node,
                                    int_node=routers[0],
                                    latency = link_latency))
            link_count += 1

        network.ext_links = ext_links

        # Create the links.
        int_links = []


        print "INTRA-group links "

        # Intra-group links (weight = 1) : 1-d flattened butterfly or fully connected
        for group in xrange(num_groups):
            for source in xrange(group_size):
                for dest in xrange(group_size):
                    if (source != dest):
                        source_id = (group * group_size) + source
                        dest_id = (group * group_size) + dest
                        print "Router " + get_id(routers[source_id]) + " created a link to Router " +  get_id(routers[dest_id])
                        int_links.append(IntLink(link_id=link_count,
                                                 src_node=routers[source_id],
                                                 dst_node=routers[dest_id],
                                                 src_outport="Intra",
                                                 dst_inport="Intra",
                                                 latency = link_latency,
                                                 weight=1))
                        link_count += 1


        
         

        print "INTER-group links " 
        print global_links_per_router

        # Inter-group links (weight = 3) : 1-d flattened butterfly or fully connected
        for source_group in xrange(num_groups):
            source_router = 0
            # Use the same source router till u exhaust the global link capacity of this router
            glink = 0
            
            for dest_group in xrange(num_groups):
                if (source_group != dest_group):
                    
                    source_id = (source_group * group_size) + source_router
                    if(dest_group < source_group):
                        dest_id = (dest_group * group_size) + int((source_group-1)/global_links_per_router)
                    else:
                        dest_id = (dest_group * group_size) + int((source_group)/global_links_per_router)
                        
                    print "Router " + get_id(routers[source_id]) + " created a link to Router " +  get_id(routers[dest_id])
                    int_links.append(IntLink(link_id=link_count,
                                             src_node=routers[source_id],
                                             dst_node=routers[dest_id],
                                             src_outport="Inter",
                                             dst_inport="Inter",
                                             latency = link_latency,
                                             weight=3))
                    link_count += 1
                    glink += 1

                    if(glink == global_links_per_router):
                        source_router += 1
                        glink = 0


        network.int_links = int_links

def get_id(node) :
    return str(node).split('.')[3].split('routers')[1]
