#! /usr/bin/perl

use strict;
use warnings;

#script to run Mesh topology with west_first (3 vcs-per-vnet) (64-cpus) (test).


my $traffic_pattern = $ARGV[0];
chomp($traffic_pattern);

my $path = '/usr/scratch/aniruddh/spin/mesh/test/multi_vc/west_first/'.$traffic_pattern;

my $sim_path = $path.'/sim';
my $network_stats_path = $path.'/network_stats.txt';

my $start_inj_rate = 0.01;
my $end_inj_rate = 1.0;
my $step = 0.05;

my $cmd = 'mkdir '.$path;
system($cmd);

$cmd = 'mkdir '.$sim_path;
system($cmd);

$cmd = '> '.$path.'/res.txt';
system($cmd);

for(my $ir=$start_inj_rate; $ir<$end_inj_rate; $ir = $ir + $step)
{
    $cmd = 'echo ------------------Sim--------------------------- >> '.$path.'/res.txt';
    system($cmd);
	
    $cmd = 'echo >> '.$path.'/res.txt';
    system($cmd);
	
    $cmd = 'echo injection rate ='.$ir.' >> '.$path.'/res.txt';
    system($cmd);

    $cmd = '/nethome/aramrakhyani3/test_spin/gem5/build/Garnet_standalone/gem5.debug -d '.$sim_path.' /nethome/aramrakhyani3/test_spin/gem5/configs/example/garnet_synth_traffic.py --network=garnet2.0 --num-cpus=64 --num-dirs=64 --topology=Mesh --mesh-rows=8 --sim-cycles=20000 --injectionrate='.$ir.' --synthetic='.$traffic_pattern.' --routing-algorithm=xy --vcs-per-vnet=3 --inj-vnet=0 > /usr/scratch/temp.txt';

    system($cmd);

    $cmd = '/nethome/aramrakhyani3/spin/gem5/my_scripts/new_extract_network_stats.sh '.$sim_path.' '.$network_stats_path;
    system($cmd);

    $cmd = 'echo >> '.$path.'/res.txt';
    system($cmd);
    
    $cmd = 'cat '.$network_stats_path.' >> '.$path.'/res.txt';
    system($cmd);

    my $file_name = $ir * 100;

    my $sim_file_inj = $sim_path.'/t'.$file_name;

    $cmd = 'cp '.$sim_path.'/stats.txt '.$sim_file_inj.'.txt';
    system($cmd);

    $cmd = 'echo >> '.$path.'/res.txt';
    system($cmd);
    
    $cmd = 'echo --------------------end------------------ >> '.$path.'/res.txt';
    system($cmd);
    
    $cmd = 'echo >> '.$path.'/res.txt';
    system($cmd);
}
