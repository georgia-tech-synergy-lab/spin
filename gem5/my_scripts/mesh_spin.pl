#! /usr/bin/perl

use strict;
use warnings;

#script to run Mesh topology with ugal and spin (3 vcs-per-vnet) (64-cpus).
#Min adaptive



my $traffic_pattern = $ARGV[0];
chomp($traffic_pattern);

my $path = '/usr/scratch/aniruddh/spin/new_results/mesh/static_bubble/'.$traffic_pattern;

my $sim_path = $path.'/sim';
my $network_stats_path = $path.'/network_stats.txt';

my $start_inj_rate = 0.001;
my $end_inj_rate = 0.2;
my $step = 0.002;

if($traffic_pattern eq 'bit_complement')
{
    $end_inj_rate = 0.064;
}
elsif($traffic_pattern eq 'bit_reverse')
{
    $end_inj_rate = 0.124;
}
elsif($traffic_pattern eq 'bit_rotation')
{
    $end_inj_rate = 0.16;
}
elsif($traffic_pattern eq 'neighbor')
{
    $end_inj_rate = 0.0;
}
elsif($traffic_pattern eq 'shuffle')
{
    $end_inj_rate = 0.143;
}
elsif($traffic_pattern eq 'tornado')
{
    $end_inj_rate = 0.11;
}
elsif($traffic_pattern eq 'transpose')
{
    $end_inj_rate = 0.135;
}
elsif($traffic_pattern eq 'uniform_random')
{
    $end_inj_rate = 0.138;
}


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

    $cmd = '/nethome/aramrakhyani3/spin/gem5/build/Garnet_standalone/gem5.debug -d '.$sim_path.' /nethome/aramrakhyani3/spin/gem5/configs/example/garnet_synth_traffic.py --network=garnet2.0 --num-cpus=64 --num-dirs=1 --topology=Mesh --mesh-rows=8 --sim-cycles=20000 --injectionrate='.$ir.' --synthetic='.$traffic_pattern.' --enable-spin-scheme=1 --dd-thresh=128 --routing-algorithm=ugal --max-turn-capacity=40 --enable-variable-dd=0 --vcs-per-vnet=2 --enable-rotating-priority=1 --enable-escape-vc=0 > '.$path.'/temp.txt';

    system($cmd);

    $cmd = '/nethome/aramrakhyani3/spin/gem5/my_scripts/new_extract_network_stats.sh '.$sim_path.' '.$network_stats_path;
    system($cmd);

    $cmd = 'echo >> '.$path.'/res.txt';
    system($cmd);
    
    $cmd = 'cat '.$network_stats_path.' >> '.$path.'/res.txt';
    system($cmd);

    my $file_name = $ir * 1000;

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
