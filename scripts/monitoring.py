#!/usr/bin/env python3
import rospy
import psutil
import time
import csv
import os

# 모니터링할 프로세스 ID(PID) 리스트
target_pids = [36577]  # 여기 원하는 PID 값을 직접 입력하세요

def monitor_process_resources():
    rospy.init_node("process_resource_monitor")
    rate = rospy.Rate(1)  # 1초마다 모니터링

    # CSV 파일 초기화 및 헤더 작성
    log_file_path = "/root/logfile(1.5).csv"
    if not os.path.exists(log_file_path):
        with open(log_file_path, "w", newline="") as csvfile:
            csv_writer = csv.writer(csvfile)
            csv_writer.writerow(["Timestamp", "PID", "CPU (%)", "Memory (MB)"])

    while not rospy.is_shutdown():
        for pid in target_pids:
            try:
                process = psutil.Process(pid)
                cpu_usage = process.cpu_percent(interval=1)
                memory_usage = process.memory_info().rss / (1024 * 1024)  # MB 단위

                rospy.loginfo("PID: {}, CPU: {}%, Memory: {} MB".format(pid, cpu_usage, memory_usage))

                # CSV 파일에 로그 기록
                with open(log_file_path, "a", newline="") as csvfile:
                    csv_writer = csv.writer(csvfile)
                    csv_writer.writerow([time.strftime("%Y-%m-%d %H:%M:%S"), pid, cpu_usage, memory_usage])

            except psutil.NoSuchProcess:
                rospy.logwarn("Process with PID {} no longer exists".format(pid))

        rate.sleep()

if __name__ == "__main__":
    try:
        monitor_process_resources()
    except rospy.ROSInterruptException:
        pass
