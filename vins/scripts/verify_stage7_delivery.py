#!/usr/bin/env python3
"""Observe the final budgeted replay's ROS publications and save an RViz window."""
import argparse
import json
import os
from pathlib import Path
import re
import subprocess
import time

import rclpy
from nav_msgs.msg import Odometry, Path as PathMessage
from sensor_msgs.msg import PointCloud
from PIL import Image
import struct
import signal


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--batch', type=Path, required=True)
    parser.add_argument('--rviz-config', type=Path, required=True)
    parser.add_argument('--timeout', type=float, default=10800)
    parser.add_argument('--acceptance-name', default='acceptance.json')
    args = parser.parse_args()
    if (args.batch/'release/ros_visualization_evidence.json').exists():
        print('existing visualization evidence retained')
        return
    if (args.batch/'release'/args.acceptance_name).exists():
        raise RuntimeError('delivery replay already finished; observe during the budgeted replay')
    deadline = time.monotonic()+args.timeout
    while not (args.batch/'release/release_manifest.json').exists():
        if time.monotonic()>deadline:
            raise RuntimeError('release was not published before deadline')
        time.sleep(1)
    release = args.batch/'release'
    topics = ['path','odometry','point_cloud','margin_cloud','key_poses','camera_pose','camera_pose_visual','keyframe_pose','keyframe_point','extrinsic','image_track']
    command = ['ros2','run','rviz2','rviz2','-d',str(args.rviz_config),'--ros-args']
    for topic in topics:
        command += ['-r',f'/vins_estimator/{topic}:=/{topic}']
    # The replay emits root topics; production launch maps these to RViz's namespace.
    log = (release/'rviz.log').open('w')
    rviz = subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    rclpy.init()
    node = rclpy.create_node('stage7_delivery_observer')
    counts = {'path':0,'odometry':0,'point_cloud':0}
    last = {}
    def callback(name):
        def receive(message):
            counts[name] += 1
            last[name] = dict(frame_id=message.header.frame_id,
                timestamp_s=message.header.stamp.sec+message.header.stamp.nanosec*1e-9)
            if name=='path':
                last[name]['pose_count'] = len(message.poses)
        return receive
    subscriptions = [node.create_subscription(cls,'/'+name,callback(name),10)
        for name,cls in [('path',PathMessage),('odometry',Odometry),('point_cloud',PointCloud)]]
    screenshot = None
    start = time.monotonic()
    last_screenshot_attempt = 0.0
    try:
        while time.monotonic()-start<180:
            rclpy.spin_once(node,timeout_sec=.1)
            if screenshot is None and counts['path']>100 and time.monotonic()-start>35 and time.monotonic()-last_screenshot_attempt>2 and rviz.poll() is None:
                last_screenshot_attempt = time.monotonic()
                tree = subprocess.check_output(['xwininfo','-root','-tree'],text=True)
                windows = re.findall(r'(0x[0-9a-f]+) "[^"\n]*RViz[^"\n]*"',tree)
                if windows:
                    geometry = subprocess.check_output(['xwininfo','-id',windows[-1]],text=True)
                    def number(label):
                        return int(re.search(label+r':\s*(-?\d+)',geometry).group(1))
                    x,y,w,h = [number(label) for label in ['Absolute upper-left X','Absolute upper-left Y','Width','Height']]
                    candidate = release/'rviz_delivery.png'
                    try:
                        raw = subprocess.check_output(['xwd','-id',windows[-1],'-silent'])
                        header = struct.unpack('>25I',raw[:100])
                        header_size,width,height,bpp,line_bytes = header[0],header[4],header[5],header[11],header[12]
                        if bpp not in (24,32):
                            raise RuntimeError('unsupported XWD pixel format')
                        pixels = raw[header_size+header[19]*12:]
                        Image.frombytes('RGB',(width,height),pixels,'raw','BGR' if bpp==24 else 'BGRX',line_bytes,1).save(candidate)
                        screenshot = candidate
                    except (OSError,ValueError,struct.error,RuntimeError,subprocess.CalledProcessError) as error:
                        print('window screenshot failed: '+str(error),flush=True)
            if (release/args.acceptance_name).exists():
                break
        evidence = dict(topic_counts=counts,last_messages=last,rviz_running=rviz.poll() is None,
            screenshot=str(screenshot) if screenshot else None,
            screenshot_note='inspect image before claiming visible trajectory',
            observer_elapsed_s=time.monotonic()-start,rviz_command=command)
        (release/'ros_visualization_evidence.json').write_text(json.dumps(evidence,indent=2)+'\n')
    finally:
        node.destroy_node()
        rclpy.shutdown()
        os.killpg(rviz.pid,signal.SIGTERM)
        try:
            rviz.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(rviz.pid,signal.SIGKILL)
        log.close()


if __name__=='__main__':
    main()
