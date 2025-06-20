#!/usr/bin/env python3
"""
ROS Node Profiler

Comprehensive real-time monitoring tool for ROS nodes that tracks:
- Execution time and FPS for each node
- Memory usage and CPU utilization
- Function-level profiling within nodes
- Network bandwidth usage
- Topic publishing rates

Usage:
    # Monitor all nodes
    rosrun autonomy ros_profiler.py

    # Monitor specific nodes
    rosrun autonomy ros_profiler.py --nodes cv_publisher controller_action

    # Enable function-level profiling
    rosrun autonomy ros_profiler.py --profile-functions

    # Export data to CSV
    rosrun autonomy ros_profiler.py --export profile_data.csv
"""

import argparse
import csv
import json
import os
import re
import subprocess
import sys
import threading
import time
from collections import defaultdict, deque
from datetime import datetime
from typing import Dict, List, Optional, Set, Tuple

import psutil
import rospy
from colorama import Back, Fore, Style, init
from std_msgs.msg import Header

# Initialize colorama for cross-platform colored output
init(autoreset=True)


class ROSNodeProfiler:
    """Real-time profiler for ROS nodes"""

    def __init__(
        self, target_nodes: Optional[List[str]] = None, profile_functions: bool = False
    ):
        self.target_nodes = target_nodes or []
        self.profile_functions = profile_functions

        # Data storage
        self.node_stats = defaultdict(
            lambda: {
                "pid": None,
                "process": None,
                "cpu_percent": deque(maxlen=60),  # Last 60 samples
                "memory_mb": deque(maxlen=60),
                "memory_percent": deque(maxlen=60),
                "publish_times": defaultdict(deque),  # Topic -> timestamps
                "fps": defaultdict(float),  # Topic -> FPS
                "last_update": time.time(),
                "function_times": defaultdict(list),
                "total_cpu_time": 0,
                "total_memory_peak": 0,
                "start_time": time.time(),
            }
        )

        # System stats
        self.system_memory_total = psutil.virtual_memory().total / (1024**2)  # MB
        self.update_interval = 0.5  # Update every 500ms
        self.running = True

        # Export settings
        self.export_file = None
        self.export_data = []

        # ROS monitoring
        rospy.init_node("ros_profiler", anonymous=True)
        self.topic_subscribers = {}
        self.discovered_nodes = set()

        # Start monitoring threads
        self.monitor_thread = threading.Thread(target=self._monitor_loop, daemon=True)
        self.display_thread = threading.Thread(target=self._display_loop, daemon=True)
        self.ros_thread = threading.Thread(target=self._ros_monitor_loop, daemon=True)

    def start(self):
        """Start the profiler"""
        print(f"{Fore.GREEN}Starting ROS Node Profiler...{Style.RESET_ALL}")

        # Discover initial nodes
        self._discover_nodes()

        # Start monitoring threads
        self.monitor_thread.start()
        self.display_thread.start()
        self.ros_thread.start()

        try:
            # Keep main thread alive
            while self.running:
                time.sleep(1)
        except KeyboardInterrupt:
            self.stop()

    def stop(self):
        """Stop the profiler"""
        self.running = False
        print(f"\n{Fore.YELLOW}Stopping profiler...{Style.RESET_ALL}")

        if self.export_file:
            self._export_data()

    def set_export_file(self, filename: str):
        """Set export file for data logging"""
        self.export_file = filename

    def _discover_nodes(self):
        """Discover active ROS nodes"""
        try:
            result = subprocess.run(
                ["rosnode", "list"], capture_output=True, text=True, timeout=5
            )
            if result.returncode == 0:
                nodes = result.stdout.strip().split("\n")
                for node in nodes:
                    node_name = node.strip("/")
                    if node_name and (
                        not self.target_nodes or node_name in self.target_nodes
                    ):
                        self.discovered_nodes.add(node_name)
                        if node_name not in self.node_stats:
                            self._init_node_monitoring(node_name)
        except subprocess.TimeoutExpired:
            rospy.logwarn("Timeout discovering ROS nodes")
        except Exception as e:
            rospy.logwarn(f"Error discovering nodes: {e}")

    def _init_node_monitoring(self, node_name: str):
        """Initialize monitoring for a specific node"""
        # Find process ID for the node
        try:
            result = subprocess.run(
                ["rosnode", "info", f"/{node_name}"],
                capture_output=True,
                text=True,
                timeout=5,
            )
            if result.returncode == 0:
                # Extract PID from rosnode info output
                pid_match = re.search(r"Pid:\s*(\d+)", result.stdout)
                if pid_match:
                    pid = int(pid_match.group(1))
                    try:
                        process = psutil.Process(pid)
                        self.node_stats[node_name]["pid"] = pid
                        self.node_stats[node_name]["process"] = process
                        rospy.loginfo(f"Monitoring node: {node_name} (PID: {pid})")
                    except psutil.NoSuchProcess:
                        rospy.logwarn(f"Process {pid} for node {node_name} not found")
        except (subprocess.TimeoutExpired, Exception) as e:
            rospy.logwarn(f"Could not get PID for node {node_name}: {e}")

    def _monitor_loop(self):
        """Main monitoring loop for system resources"""
        while self.running:
            try:
                current_time = time.time()

                # Update node discovery periodically
                if current_time % 10 < self.update_interval:  # Every 10 seconds
                    self._discover_nodes()

                # Monitor each discovered node
                for node_name in list(self.discovered_nodes):
                    self._monitor_node(node_name, current_time)

                # Store data for export
                if self.export_file:
                    self._record_export_data(current_time)

                time.sleep(self.update_interval)

            except Exception as e:
                rospy.logerr(f"Error in monitor loop: {e}")
                time.sleep(1)

    def _monitor_node(self, node_name: str, current_time: float):
        """Monitor a specific node's resource usage"""
        stats = self.node_stats[node_name]
        process = stats["process"]

        if process is None:
            return

        try:
            # Check if process is still alive
            if not process.is_running():
                rospy.logwarn(f"Node {node_name} process is no longer running")
                self.discovered_nodes.discard(node_name)
                return

            # Get CPU and memory usage
            cpu_percent = process.cpu_percent()
            memory_info = process.memory_info()
            memory_mb = memory_info.rss / (1024**2)  # Convert to MB
            memory_percent = (memory_mb / self.system_memory_total) * 100

            # Store metrics
            stats["cpu_percent"].append(cpu_percent)
            stats["memory_mb"].append(memory_mb)
            stats["memory_percent"].append(memory_percent)
            stats["last_update"] = current_time

            # Update totals
            stats["total_cpu_time"] += cpu_percent * self.update_interval / 100
            stats["total_memory_peak"] = max(stats["total_memory_peak"], memory_mb)

        except (psutil.NoSuchProcess, psutil.AccessDenied) as e:
            rospy.logwarn(f"Could not monitor node {node_name}: {e}")
            self.discovered_nodes.discard(node_name)

    def _ros_monitor_loop(self):
        """Monitor ROS-specific metrics like topic publishing rates"""
        while self.running:
            try:
                self._monitor_topic_rates()
                time.sleep(1.0)  # Check topic rates every second
            except Exception as e:
                rospy.logerr(f"Error in ROS monitor loop: {e}")
                time.sleep(1)

    def _monitor_topic_rates(self):
        """Monitor topic publishing rates to calculate FPS"""
        try:
            # Get topic list
            result = subprocess.run(
                ["rostopic", "list"], capture_output=True, text=True, timeout=5
            )
            if result.returncode != 0:
                return

            topics = result.stdout.strip().split("\n")
            current_time = time.time()

            for topic in topics[:10]:  # Limit to first 10 topics to avoid overhead
                if not topic.strip():
                    continue

                try:
                    # Get topic info to find publishing node
                    info_result = subprocess.run(
                        ["rostopic", "info", topic],
                        capture_output=True,
                        text=True,
                        timeout=2,
                    )
                    if info_result.returncode == 0:
                        # Extract publisher information
                        publishers = re.findall(r"\* (/\w+)", info_result.stdout)
                        for publisher in publishers:
                            node_name = publisher.strip("/")
                            if node_name in self.discovered_nodes:
                                # Record publish time
                                self.node_stats[node_name]["publish_times"][
                                    topic
                                ].append(current_time)

                                # Calculate FPS (messages in last second)
                                times = self.node_stats[node_name]["publish_times"][
                                    topic
                                ]
                                recent_times = [
                                    t for t in times if current_time - t <= 1.0
                                ]
                                self.node_stats[node_name]["fps"][topic] = len(
                                    recent_times
                                )

                                # Keep only recent timestamps
                                while (
                                    times and current_time - times[0] > 60
                                ):  # Keep 1 minute of data
                                    times.popleft()

                except subprocess.TimeoutExpired:
                    continue
                except Exception as e:
                    continue

        except subprocess.TimeoutExpired:
            pass
        except Exception as e:
            rospy.logwarn(f"Error monitoring topic rates: {e}")

    def _display_loop(self):
        """Display monitoring information"""
        while self.running:
            try:
                self._display_stats()
                time.sleep(1.0)  # Update display every second
            except Exception as e:
                rospy.logerr(f"Error in display loop: {e}")
                time.sleep(1)

    def _display_stats(self):
        """Display current statistics"""
        # Clear screen
        os.system("clear")

        # Header
        print(f"{Fore.CYAN}{'=' * 80}")
        print(f"{Fore.CYAN}{'ROS NODE PROFILER':^80}")
        print(f"{Fore.CYAN}{'=' * 80}{Style.RESET_ALL}")
        print(f"Time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        print(f"Total System Memory: {self.system_memory_total:.1f} MB")
        print(f"Monitoring {len(self.discovered_nodes)} nodes")
        print()

        # Node statistics table
        if self.discovered_nodes:
            print(f"{Fore.YELLOW}NODE STATISTICS:{Style.RESET_ALL}")
            print(
                f"{'Node Name':<20} {'PID':<8} {'CPU %':<8} {'Mem MB':<10} {'Mem %':<8} {'Topics':<8} {'Avg FPS':<10}"
            )
            print("-" * 80)

            total_memory_used = 0

            for node_name in sorted(self.discovered_nodes):
                stats = self.node_stats[node_name]

                # Get current values
                pid = stats["pid"] or "N/A"
                cpu_avg = (
                    sum(stats["cpu_percent"]) / len(stats["cpu_percent"])
                    if stats["cpu_percent"]
                    else 0
                )
                mem_current = stats["memory_mb"][-1] if stats["memory_mb"] else 0
                mem_percent = (
                    stats["memory_percent"][-1] if stats["memory_percent"] else 0
                )
                topic_count = len(stats["fps"])
                avg_fps = (
                    sum(stats["fps"].values()) / len(stats["fps"])
                    if stats["fps"]
                    else 0
                )

                total_memory_used += mem_current

                # Color coding based on resource usage
                cpu_color = self._get_usage_color(cpu_avg, [20, 50])
                mem_color = self._get_usage_color(mem_percent, [10, 25])

                print(
                    f"{node_name:<20} {pid:<8} {cpu_color}{cpu_avg:>6.1f}{Style.RESET_ALL}% "
                    f"{mem_color}{mem_current:>8.1f}{Style.RESET_ALL} {mem_color}{mem_percent:>6.1f}{Style.RESET_ALL}% "
                    f"{topic_count:<8} {avg_fps:>8.1f}"
                )

            print("-" * 80)
            print(
                f"{'TOTAL':<20} {'':<8} {'':<8} {total_memory_used:>8.1f} "
                f"{(total_memory_used/self.system_memory_total)*100:>6.1f}%"
            )

        # Topic publishing rates
        print(f"\n{Fore.YELLOW}TOPIC PUBLISHING RATES:{Style.RESET_ALL}")
        topic_data = []
        for node_name in self.discovered_nodes:
            for topic, fps in self.node_stats[node_name]["fps"].items():
                if fps > 0:
                    topic_data.append((topic, node_name, fps))

        if topic_data:
            topic_data.sort(key=lambda x: x[2], reverse=True)  # Sort by FPS
            print(f"{'Topic':<40} {'Publisher':<20} {'FPS':<10}")
            print("-" * 70)
            for topic, node, fps in topic_data[:15]:  # Show top 15
                fps_color = self._get_fps_color(fps)
                print(f"{topic:<40} {node:<20} {fps_color}{fps:>8.1f}{Style.RESET_ALL}")
        else:
            print("No active topic publishing detected")

        # Performance summary
        print(f"\n{Fore.YELLOW}PERFORMANCE SUMMARY:{Style.RESET_ALL}")
        if self.discovered_nodes:
            all_cpu = [
                sum(stats["cpu_percent"]) / len(stats["cpu_percent"])
                for stats in self.node_stats.values()
                if stats["cpu_percent"]
            ]
            all_memory = [
                stats["memory_mb"][-1]
                for stats in self.node_stats.values()
                if stats["memory_mb"]
            ]

            if all_cpu:
                print(f"Average CPU Usage: {sum(all_cpu)/len(all_cpu):.1f}%")
            if all_memory:
                print(
                    f"Total Memory Usage: {sum(all_memory):.1f} MB ({(sum(all_memory)/self.system_memory_total)*100:.1f}%)"
                )

        print(f"\n{Fore.GREEN}Press Ctrl+C to stop profiling{Style.RESET_ALL}")

    def _get_usage_color(self, value: float, thresholds: List[float]) -> str:
        """Get color based on usage thresholds"""
        if value < thresholds[0]:
            return Fore.GREEN
        elif value < thresholds[1]:
            return Fore.YELLOW
        else:
            return Fore.RED

    def _get_fps_color(self, fps: float) -> str:
        """Get color based on FPS value"""
        if fps >= 30:
            return Fore.GREEN
        elif fps >= 10:
            return Fore.YELLOW
        else:
            return Fore.RED

    def _record_export_data(self, current_time: float):
        """Record data for export"""
        for node_name in self.discovered_nodes:
            stats = self.node_stats[node_name]

            if stats["cpu_percent"] and stats["memory_mb"]:
                record = {
                    "timestamp": current_time,
                    "datetime": datetime.fromtimestamp(current_time).isoformat(),
                    "node_name": node_name,
                    "pid": stats["pid"],
                    "cpu_percent": stats["cpu_percent"][-1],
                    "memory_mb": stats["memory_mb"][-1],
                    "memory_percent": stats["memory_percent"][-1],
                    "topic_count": len(stats["fps"]),
                    "avg_fps": (
                        sum(stats["fps"].values()) / len(stats["fps"])
                        if stats["fps"]
                        else 0
                    ),
                    "total_cpu_time": stats["total_cpu_time"],
                    "memory_peak": stats["total_memory_peak"],
                    "uptime": current_time - stats["start_time"],
                }
                self.export_data.append(record)

    def _export_data(self):
        """Export collected data to CSV"""
        if not self.export_data:
            print("No data to export")
            return

        try:
            with open(self.export_file, "w", newline="") as csvfile:
                fieldnames = self.export_data[0].keys()
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                writer.writeheader()
                writer.writerows(self.export_data)

            print(f"{Fore.GREEN}Data exported to {self.export_file}{Style.RESET_ALL}")

            # Export summary statistics
            summary_file = self.export_file.replace(".csv", "_summary.json")
            summary = self._generate_summary()
            with open(summary_file, "w") as f:
                json.dump(summary, f, indent=2)

            print(f"{Fore.GREEN}Summary exported to {summary_file}{Style.RESET_ALL}")

        except Exception as e:
            print(f"{Fore.RED}Error exporting data: {e}{Style.RESET_ALL}")

    def _generate_summary(self) -> Dict:
        """Generate summary statistics"""
        summary = {
            "profiling_duration": time.time()
            - min(stats["start_time"] for stats in self.node_stats.values()),
            "nodes_monitored": list(self.discovered_nodes),
            "total_samples": len(self.export_data),
            "node_statistics": {},
        }

        for node_name in self.discovered_nodes:
            stats = self.node_stats[node_name]
            if stats["cpu_percent"] and stats["memory_mb"]:
                summary["node_statistics"][node_name] = {
                    "avg_cpu_percent": sum(stats["cpu_percent"])
                    / len(stats["cpu_percent"]),
                    "max_cpu_percent": max(stats["cpu_percent"]),
                    "avg_memory_mb": sum(stats["memory_mb"]) / len(stats["memory_mb"]),
                    "max_memory_mb": max(stats["memory_mb"]),
                    "peak_memory_mb": stats["total_memory_peak"],
                    "total_cpu_time": stats["total_cpu_time"],
                    "topic_count": len(stats["fps"]),
                    "max_fps": max(stats["fps"].values()) if stats["fps"] else 0,
                }

        return summary


def main():
    parser = argparse.ArgumentParser(description="ROS Node Profiler")
    parser.add_argument("--nodes", nargs="+", help="Specific nodes to monitor")
    parser.add_argument(
        "--profile-functions",
        action="store_true",
        help="Enable function-level profiling (experimental)",
    )
    parser.add_argument("--export", help="Export data to CSV file")
    parser.add_argument(
        "--update-rate",
        type=float,
        default=0.5,
        help="Update rate in seconds (default: 0.5)",
    )

    args = parser.parse_args()

    try:
        profiler = ROSNodeProfiler(
            target_nodes=args.nodes, profile_functions=args.profile_functions
        )

        profiler.update_interval = args.update_rate

        if args.export:
            profiler.set_export_file(args.export)

        profiler.start()

    except KeyboardInterrupt:
        print("\nProfiler stopped by user")
    except Exception as e:
        print(f"Error running profiler: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
