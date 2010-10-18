#!/bin/bash

rosrun xacro xacro.py irp6p_fakeservo.xml.xacro -o example.xml
`rospack find ocl`/bin/deployer-gnulinux -s example.xml
