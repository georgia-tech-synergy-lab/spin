echo > $2
grep "packets_injected::total" $1/stats.txt | sed 's/system.ruby.network.packets_injected::total\s*/packets_injected = /' >> $2
grep "packets_received::total" $1/stats.txt | sed 's/system.ruby.network.packets_received::total\s*/packets_received = /' >> $2
grep "average_packet_queueing_latency" $1/stats.txt | sed 's/system.ruby.network.average_packet_queueing_latency\s*/average_packet_queueing_latency = /' >> $2
grep "average_packet_network_latency" $1/stats.txt | sed 's/system.ruby.network.average_packet_network_latency\s*/average_packet_network_latency = /' >> $2
grep "average_packet_latency" $1/stats.txt | sed 's/system.ruby.network.average_packet_latency\s*/average_packet_latency = /' >> $2
grep "flits_injected::total" $1/stats.txt | sed 's/system.ruby.network.flits_injected::total\s*/flits_injected = /' >> $2
grep "flits_received::total" $1/stats.txt | sed 's/system.ruby.network.flits_received::total\s*/flits_received = /' >> $2
grep "average_flit_queueing_latency" $1/stats.txt | sed 's/system.ruby.network.average_flit_queueing_latency\s*/average_flit_queueing_latency = /' >> $2
grep "average_flit_network_latency" $1/stats.txt | sed 's/system.ruby.network.average_flit_network_latency\s*/average_flit_network_latency = /' >> $2
grep "average_flit_latency" $1/stats.txt | sed 's/system.ruby.network.average_flit_latency\s*/average_flit_latency = /' >> $2
grep "average_hops" $1/stats.txt | sed 's/system.ruby.network.average_hops\s*/average_hops = /' >> $2

grep "total_probes_sent" $1/stats.txt | sed 's/system.ruby.network.total_probes_sent\s*/total_probes_sent = /' >> $2
grep "total_move_sent" $1/stats.txt | sed 's/system.ruby.network.total_move_sent\s*/total_move_sent = /' >> $2
grep "total_kill_move_sent" $1/stats.txt | sed 's/system.ruby.network.total_kill_move_sent\s*/total_kill_move_sent = /' >> $2
grep "total_check_probe_sent" $1/stats.txt | sed 's/system.ruby.network.total_check_probe_sent\s*/total_check_probe_sent = /' >> $2

grep "total_probes_dropped" $1/stats.txt | sed 's/system.ruby.network.total_probes_dropped\s*/total_probes_dropped = /' >> $2
grep "total_move_dropped" $1/stats.txt | sed 's/system.ruby.network.total_move_dropped\s*/total_move_dropped = /' >> $2
grep "total_kill_move_dropped" $1/stats.txt | sed 's/system.ruby.network.total_kill_move_dropped\s*/total_kill_move_dropped = /' >> $2
grep "total_check_probe_dropped" $1/stats.txt | sed 's/system.ruby.network.total_check_probe_dropped\s*/total_check_probe_dropped = /' >> $2

grep "total_spins" $1/stats.txt | sed 's/system.ruby.network.total_spins\s*/total_spins = /' >> $2
grep "total_spin_cycles" $1/stats.txt | sed 's/system.ruby.network.total_spin_cycles\s*/total_spin_cycles = /' >> $2
grep "network_max_spin_cycles" $1/stats.txt | sed 's/system.ruby.network.network_max_spin_cycles\s*/network_max_spin_cycles = /' >> $2

grep "network_max_deadlock_path_length" $1/stats.txt | sed 's/system.ruby.network.network_max_deadlock_path_length\s*/network_max_deadlock_path_length = /' >> $2

grep "network_deadlock_path_length_sum" $1/stats.txt | sed 's/system.ruby.network.network_deadlock_path_length_sum\s*/network_deadlock_path_length_sum = /' >> $2

grep "probe_link_utilisation" $1/stats.txt | sed 's/system.ruby.network.probe_link_utilisation\s*/probe_link_utilisation = /' >> $2
grep "move_link_utilisation" $1/stats.txt | sed 's/system.ruby.network.move_link_utilisation\s*/move_link_utilisation = /' >> $2
grep "kill_move_link_utilisation" $1/stats.txt | sed 's/system.ruby.network.kill_move_link_utilisation\s*/kill_move_link_utilisation = /' >> $2
grep "check_probe_link_utilisation" $1/stats.txt | sed 's/system.ruby.network.check_probe_link_utilisation\s*/check_probe_link_utilisation = /' >> $2

grep "flit_link_utilisation" $1/stats.txt | sed 's/system.ruby.network.flit_link_utilisation\s*/flit_link_utilisation = /' >> $2
grep "link_utilization" $1/stats.txt | sed 's/system.ruby.network.link_utilization\s*/link_utilization = /' >> $2

