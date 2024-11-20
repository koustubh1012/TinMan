# TinMan

## Badges

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![codecov](https://codecov.io/gh/koustubh1012/TinMan/graph/badge.svg?token=XTWB4FQJ5K)](https://codecov.io/gh/koustubh1012/TinMan)
![CICD Workflow status](https://github.com/koustubh1012/TinMan/actions/workflows/run-unit-test-and-upload-codecov.yml/badge.svg)

## Acme Robotics (Tinman) A Mobile Autonomous Robot for Collection of Waste in Cafeteria

## Overview

TinMan is an innovative autonomous robot designed to revolutionize waste management in cafeteria settings by collecting discarded aluminum cans. Developed by ACME Robotics, TinMan is an efficient, hygienic, and sustainable solution that reduces labor costs, promotes recycling, and addresses the challenges of dynamic and cluttered environments.

## Authors

1. FNU Koustubh             (120273766, koustubh@umd.edu)
2. Keyur Borad              (120426049, kborad@umd.edu)
3. Swaraj Mundruppady Rao   (120127007, swarajmr@umd.edu)

## Purpose

Cafeterias generate significant amounts of recyclable waste, particularly aluminum cans. Manual collection is inefficient, time-consuming, and unhygienic. TinMan autonomously navigates the cafeteria, identifies and collects scattered cans, and disposes of them at a designated area once its onboard collection bin is full.

## Product Backlog

For detailed project backlog document, refer to the following document: [Project backlog Document](https://docs.google.com/spreadsheets/d/15zRh9hyb8FhVGP8c3GUeDephmkg42Pm05zHLUwGk4No/edit?gid=0#gid=0)

## Sprint Planning Notes

For detailed sprint planning notes, refer to the following document: [Sprint Planning Notes](https://docs.google.com/document/d/1aYkBTQEc9sz_2KH6B-emRayM40f5MsLR75mS5LxTISY/edit?tab=t.0#heading=h.6j90akwcnl1i)

## Quad Chart

For detailed quad chart, refer to the following presentation: [Quad Chart](https://docs.google.com/presentation/d/1e9iCOqxLyKkk5ClS3eGCo465Qvf01dGejnwnA9A_B3Q/edit#slide=id.g316a9aed667_2_89)

## Developer Documentation

### Dependencies

1. ROS2 Humble: ROS2 Humble installed on a system with Ubuntu 22.04. Follow the instructions on this [website](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) to install ROS2 Humble.

2. OpenCV: Follow the instructions on this [website](https://www.geeksforgeeks.org/how-to-install-opencv-in-c-on-linux/) to install OpenCV. This is required to detect the cans using classical image processing algorithms

3. [Gazebo](https://classic.gazebosim.org/tutorials?tut=install_ubuntu) : Gazebo latest version was installed for deploying simulation environments and running the models in the project package. [website]
