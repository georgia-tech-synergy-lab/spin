echo > result_temp.txt
grep -w "average_flit_latency" /usr/scratch/aniruddh/spin/new_results/mesh/static_bubble/uniform_random/res.txt | sed 's/average_flit_latency = \s*//' >> result_temp.txt
#grep -w "injection rate" /usr/scratch/aniruddh/spin/new_results/mesh/mixed/escapevc/bit_complement/res.txt | sed 's/injection rate =\s*//' >> result_temp.txt
