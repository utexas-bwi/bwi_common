#!/bin/bash
hostname=$(hostname)
link="http://nixons-head.csres.utexas.edu/tours"
to="your.email@gmail.com"

echo -e "Tour started on $hostname\nCheck it out at: $link" | mail -s "[bwi_virtour] Tour started on $hostname" "$to"
