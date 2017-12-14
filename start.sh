#!/usr/bin/env bash

sudo modprobe can
sudo modprobe can-raw
sudo modprobe can-dev
sudo modprobe vcan
sudo modprobe kvaser_usb

sudo ip link add dev hs1 type vcan
sudo ip link add dev hs2 type vcan
sudo ip link add dev hs3 type vcan
sudo ip link add dev ms1 type vcan

sudo ip link set up hs1
sudo ip link set up hs2
sudo ip link set up hs3
sudo ip link set up ms1

/usr/bin/tmux new-session -s fakecan -d canplayer -l i -I /home/jefferson/sdc-start/80can.log hs1=can0 hs2=can1 hs3=can2
