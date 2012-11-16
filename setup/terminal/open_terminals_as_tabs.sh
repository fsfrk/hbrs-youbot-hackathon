#!/bin/bash
if [ -n "${1}" ]; then
    gnome-terminal \
    --tab 	-e "bash -c 'ssh -X ${1}'" \
    --tab	-e "bash -c 'ssh -X ${1}'" \
    --tab	-e "bash -c 'ssh -X ${1}'" \
    --tab	-e "bash -c 'ssh -X ${1}'"
else
    gnome-terminal --working-directory=$(pwd) \
    --tab \
    --tab \
    --tab \
    --tab    
fi
