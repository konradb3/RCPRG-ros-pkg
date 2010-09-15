#!/bin/bash

rosrun xacro xacro.py example.xml.xacro -o example.xml
`rospack find ocl`/bin/deployer-gnulinux -s example.xml
