#!/bin/bash
if [[ $(/usr/bin/id -u) -ne 0 ]]; then
  echo "Needs to be run as root"
  exit
fi
if [ ! -f /etc/hosts_original ]; then
  cp /etc/hosts /etc/hosts_original
fi
cat /etc/hosts_original > /etc/hosts
curl -s http://nixons-head.csres.utexas.edu:7979/hosts >> /etc/hosts
