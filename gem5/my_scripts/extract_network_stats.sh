echo > network_stats.txt
grep "packets_injected::total" m5out/stats.txt | sed 's/system.ruby.network.packets_injected::total\s*/packets_injected = /' >> network_stats.txt
grep "packets_received::total" m5out/stats.txt | sed 's/system.ruby.network.packets_received::total\s*/packets_received = /' >> network_stats.txt
grep "average_packet_queueing_latency" m5out/stats.txt | sed 's/system.ruby.network.average_packet_queueing_latency\s*/average_packet_queueing_latency = /' >> network_stats.txt
grep "average_packet_network_latency" m5out/stats.txt | sed 's/system.ruby.network.average_packet_network_latency\s*/average_packet_network_latency = /' >> network_stats.txt
grep "average_packet_latency" m5out/stats.txt | sed 's/system.ruby.network.average_packet_latency\s*/average_packet_latency = /' >> network_stats.txt
grep "flits_injected::total" m5out/stats.txt | sed 's/system.ruby.network.flits_injected::total\s*/flits_injected = /' >> network_stats.txt
grep "flits_received::total" m5out/stats.txt | sed 's/system.ruby.network.flits_received::total\s*/flits_received = /' >> network_stats.txt
grep "average_flit_queueing_latency" m5out/stats.txt | sed 's/system.ruby.network.average_flit_queueing_latency\s*/average_flit_queueing_latency = /' >> network_stats.txt
grep "average_flit_network_latency" m5out/stats.txt | sed 's/system.ruby.network.average_flit_network_latency\s*/average_flit_network_latency = /' >> network_stats.txt
grep "average_flit_latency" m5out/stats.txt | sed 's/system.ruby.network.average_flit_latency\s*/average_flit_latency = /' >> network_stats.txt
grep "average_hops" m5out/stats.txt | sed 's/system.ruby.network.average_hops\s*/average_hops = /' >> network_stats.txt

grep "total_probes_sent" m5out/stats.txt | sed 's/system.ruby.network.total_probes_sent\s*/total_probes_sent = /' >> network_stats.txt
grep "total_move_sent" m5out/stats.txt | sed 's/system.ruby.network.total_move_sent\s*/total_move_sent = /' >> network_stats.txt
grep "total_kill_move_sent" m5out/stats.txt | sed 's/system.ruby.network.total_kill_move_sent\s*/total_kill_move_sent = /' >> network_stats.txt
grep "total_check_probe_sent" m5out/stats.txt | sed 's/system.ruby.network.total_check_probe_sent\s*/total_check_probe_sent = /' >> network_stats.txt

grep "total_probes_dropped" m5out/stats.txt | sed 's/system.ruby.network.total_probes_dropped\s*/total_probes_dropped = /' >> network_stats.txt
grep "total_move_dropped" m5out/stats.txt | sed 's/system.ruby.network.total_move_dropped\s*/total_move_dropped = /' >> network_stats.txt
grep "total_kill_move_dropped" m5out/stats.txt | sed 's/system.ruby.network.total_kill_move_dropped\s*/total_kill_move_dropped = /' >> network_stats.txt
grep "total_check_probe_dropped" m5out/stats.txt | sed 's/system.ruby.network.total_check_probe_dropped\s*/total_check_probe_dropped = /' >> network_stats.txt

grep "total_spins" m5out/stats.txt | sed 's/system.ruby.network.total_spins\s*/total_spins = /' >> network_stats.txt
grep "total_spin_cycles" m5out/stats.txt | sed 's/system.ruby.network.total_spin_cycles\s*/total_spin_cycles = /' >> network_stats.txt
grep "network_max_spin_cycles" m5out/stats.txt | sed 's/system.ruby.network.network_max_spin_cycles\s*/network_max_spin_cycles = /' >> network_stats.txt

grep "network_max_deadlock_path_length" m5out/stats.txt | sed 's/system.ruby.network.network_max_deadlock_path_length\s*/network_max_deadlock_path_length = /' >> network_stats.txt

grep "network_deadlock_path_length_sum" m5out/stats.txt | sed 's/system.ruby.network.network_deadlock_path_length_sum\s*/network_deadlock_path_length_sum = /' >> network_stats.txt

grep "probe_link_utilisation" m5out/stats.txt | sed 's/system.ruby.network.probe_link_utilisation\s*/probe_link_utilisation = /' >> network_stats.txt
grep "move_link_utilisation" m5out/stats.txt | sed 's/system.ruby.network.move_link_utilisation\s*/move_link_utilisation = /' >> network_stats.txt
grep "kill_move_link_utilisation" m5out/stats.txt | sed 's/system.ruby.network.kill_move_link_utilisation\s*/kill_move_link_utilisation = /' >> network_stats.txt
grep "check_probe_link_utilisation" m5out/stats.txt | sed 's/system.ruby.network.check_probe_link_utilisation\s*/check_probe_link_utilisation = /' >> network_stats.txt

grep "flit_link_utilisation" m5out/stats.txt | sed 's/system.ruby.network.flit_link_utilisation\s*/flit_link_utilisation = /' >> network_stats.txt
grep "link_utilization" m5out/stats.txt | sed 's/system.ruby.network.link_utilization\s*/link_utilization = /' >> network_stats.txt

