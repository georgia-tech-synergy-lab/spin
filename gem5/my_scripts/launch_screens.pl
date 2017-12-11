#! /usr/bin/perl

use strict;
use warnings;

my $scr_name = 'mesh_sb';

my @traffic_patterns = ('uniform_random', 'tornado', 'bit_complement', 'bit_reverse', 'bit_rotation', 'shuffle', 'transpose');

#my @traffic_patterns = ('neighbor');

for(my  $iter=0; $iter<7; $iter++)
{
    my  $traffic = $traffic_patterns[$iter];
    
    my $cmd = 'screen -m -d -S '.$scr_name.$traffic;
    system($cmd);

    $cmd = 'screen -S '.$scr_name.$traffic.' -p 0 -X stuff "tcsh
"';
    system($cmd); 

    $cmd = 'screen -S '.$scr_name.$traffic.' -p 0 -X stuff "source /nethome/aramrakhyani3/spin/gem5/my_scripts/set_env.cshrc
"';
    system($cmd);

    $cmd = 'screen -S '.$scr_name.$traffic.' -p 0 -X stuff "perl /nethome/aramrakhyani3/spin/gem5/my_scripts/mesh_spin.pl '.$traffic.'
"';
    system($cmd);
}
