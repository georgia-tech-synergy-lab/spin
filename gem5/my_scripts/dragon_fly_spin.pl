#! /usr/bin/perl

use strict;
use warnings;

#script to run dragon fly topology with ugal and spin (1 vcs-per-vnet) (1024-cpus).


my $traffic_pattern = $ARGV[0];
chomp($traffic_pattern);

my $path = '/usr/scratch/aniruddh/spin/new_results/dfly/single_vc/minimal/'.$traffic_pattern;

my $sim_path = $path.'/sim';
my $network_stats_path = $path.'/network_stats.txt';

my $start_inj_rate = 0.01;
my $end_inj_rate = 0.3;
my $step = 0.002;


if($traffic_pattern eq 'bit_complement')
{
    $end_inj_rate = 0.22;
}
elsif($traffic_pattern eq 'bit_reverse')
{
    $end_inj_rate = 0.27;
}
elsif($traffic_pattern eq 'bit_rotation')
{
    $end_inj_rate = 0.22;
}
elsif($traffic_pattern eq 'neighbor')
{
    $start_inj_rate = 0.274;
    $end_inj_rate = 0.32;
}
elsif($traffic_pattern eq 'shuffle')
{
    $end_inj_rate = 0.22;
}
elsif($traffic_pattern eq 'tornado')
{
    $end_inj_rate = 0.17;
}
elsif($traffic_pattern eq 'transpose')
{
    $start_inj_rate = 0.252;
    $end_inj_rate = 0.32;
}
elsif($traffic_pattern eq 'uniform_random')
{
    $start_inj_rate = 0.22;
    $end_inj_rate = 0.26;
}


my $cmd = 'mkdir '.$path;
system($cmd);

$cmd = 'mkdir '.$sim_path;
system($cmd);

$cmd = '> '.$path.'/res1.txt';
system($cmd);

for(my $ir=$start_inj_rate; $ir<$end_inj_rate; $ir = $ir + $step)
{
    $cmd = 'echo ------------------Sim--------------------------- >> '.$path.'/res1.txt';
    system($cmd);
	
    $cmd = 'echo >> '.$path.'/res1.txt';
    system($cmd);
	
    $cmd = 'echo injection rate ='.$ir.' >> '.$path.'/res1.txt';
    system($cmd);

    $cmd = '/nethome/aramrakhyani3/spin/gem5/build/Garnet_standalone/gem5.debug -d '.$sim_path.' /nethome/aramrakhyani3/spin/gem5/configs/example/garnet_synth_traffic.py --network=garnet2.0 --num-cpus=1024 --num-dirs=1 --topology=dragonfly --dfly-group-size=8 --sim-cycles=20000 --injectionrate='.$ir.' --synthetic='.$traffic_pattern.' --enable-spin-scheme=1 --dd-thresh=128 --routing-algorithm=table --max-turn-capacity=50 --enable-variable-dd=0 --vcs-per-vnet=1 --enable-rotating-priority=1 --enable-dfly-dlock-avoidance=0 > '.$path.'/temp.txt';

    system($cmd);

    $cmd = '/nethome/aramrakhyani3/spin/gem5/my_scripts/new_extract_network_stats.sh '.$sim_path.' '.$network_stats_path;
    system($cmd);

    $cmd = 'echo >> '.$path.'/res1.txt';
    system($cmd);
    
    $cmd = 'cat '.$network_stats_path.' >> '.$path.'/res1.txt';
    system($cmd);

    my $file_name = $ir * 100;

    my $sim_file_inj = $sim_path.'/t'.$file_name;

    $cmd = 'cp '.$sim_path.'/stats.txt '.$sim_file_inj.'.txt';
    system($cmd);

    $cmd = 'echo >> '.$path.'/res1.txt';
    system($cmd);
    
    $cmd = 'echo --------------------end------------------ >> '.$path.'/res1.txt';
    system($cmd);
    
    $cmd = 'echo >> '.$path.'/res1.txt';
    system($cmd);
}
