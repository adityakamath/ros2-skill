#!/usr/bin/env python3
"""Performance telemetry and timing instrumentation for ros2-skill.

This module provides utilities for tracking command execution timing,
profiling hot paths, and identifying performance bottlenecks.
"""

import functools
import json
import os
import time
from typing import Any, Callable, Dict, List, Optional


class TimingCollector:
    """Collects timing data for command execution phases."""
    
    def __init__(self):
        self.phases: List[Dict[str, Any]] = []
        self.start_time: Optional[float] = None
        self.current_phase: Optional[str] = None
        self.phase_start: Optional[float] = None
    
    def start(self, command: str):
        """Start timing collection for a command."""
        self.start_time = time.time()
        self.phases = []
        self.current_phase = None
        self.phase_start = None
        self.phases.append({
            "phase": "start",
            "command": command,
            "timestamp": self.start_time,
        })
    
    def phase(self, name: str):
        """Record start of a new phase."""
        now = time.time()
        
        # End previous phase
        if self.current_phase and self.phase_start:
            elapsed = now - self.phase_start
            self.phases.append({
                "phase": self.current_phase,
                "elapsed_s": elapsed,
                "timestamp": now,
            })
        
        # Start new phase
        self.current_phase = name
        self.phase_start = now
    
    def end(self) -> Dict[str, Any]:
        """End timing collection and return summary."""
        now = time.time()
        
        # End current phase
        if self.current_phase and self.phase_start:
            elapsed = now - self.phase_start
            self.phases.append({
                "phase": self.current_phase,
                "elapsed_s": elapsed,
                "timestamp": now,
            })
        
        # Calculate total
        total_elapsed = now - self.start_time if self.start_time else 0.0
        
        return {
            "total_elapsed_s": total_elapsed,
            "phases": self.phases,
            "phase_summary": self._summarize_phases(),
        }
    
    def _summarize_phases(self) -> Dict[str, float]:
        """Summarize phase timings."""
        summary = {}
        for phase in self.phases:
            if "elapsed_s" in phase:
                name = phase["phase"]
                summary[name] = phase["elapsed_s"]
        return summary


# Global timing collector (optional, enabled via environment variable)
_TELEMETRY_ENABLED = os.environ.get("ROS2_SKILL_TELEMETRY", "0") == "1"
_timing_collector: Optional[TimingCollector] = None


def enable_telemetry():
    """Enable telemetry collection globally."""
    global _TELEMETRY_ENABLED
    _TELEMETRY_ENABLED = True


def disable_telemetry():
    """Disable telemetry collection globally."""
    global _TELEMETRY_ENABLED
    _TELEMETRY_ENABLED = False


def get_timing_collector() -> Optional[TimingCollector]:
    """Get current timing collector."""
    return _timing_collector


def start_timing(command: str) -> Optional[TimingCollector]:
    """Start timing collection for a command."""
    global _timing_collector
    if _TELEMETRY_ENABLED:
        _timing_collector = TimingCollector()
        _timing_collector.start(command)
        return _timing_collector
    return None


def phase(name: str):
    """Record a timing phase."""
    if _timing_collector:
        _timing_collector.phase(name)


def end_timing() -> Optional[Dict[str, Any]]:
    """End timing collection and return results."""
    global _timing_collector
    if _timing_collector:
        result = _timing_collector.end()
        _timing_collector = None
        return result
    return None


def timed(phase_name: Optional[str] = None):
    """Decorator to time a function execution.
    
    Usage:
        @timed("load_config")
        def load_config():
            ...
    """
    def decorator(func: Callable) -> Callable:
        name = phase_name or func.__name__
        
        @functools.wraps(func)
        def wrapper(*args, **kwargs):
            if _timing_collector:
                _timing_collector.phase(name)
                result = func(*args, **kwargs)
                _timing_collector.phase(f"{name}_complete")
                return result
            else:
                return func(*args, **kwargs)
        
        return wrapper
    return decorator


class PerformanceProfiler:
    """Profile performance of command execution."""
    
    def __init__(self):
        self.measurements: List[Dict[str, Any]] = []
    
    def record(self, command: str, timing: Dict[str, Any], metadata: Optional[Dict[str, Any]] = None):
        """Record a performance measurement."""
        measurement = {
            "command": command,
            "timestamp": time.time(),
            "timing": timing,
        }
        if metadata:
            measurement["metadata"] = metadata
        
        self.measurements.append(measurement)
    
    def get_stats(self, command: Optional[str] = None) -> Dict[str, Any]:
        """Get statistics for recorded measurements."""
        filtered = self.measurements
        if command:
            filtered = [m for m in self.measurements if m["command"] == command]
        
        if not filtered:
            return {"count": 0}
        
        total_times = [m["timing"]["total_elapsed_s"] for m in filtered]
        
        return {
            "count": len(filtered),
            "total_avg_s": sum(total_times) / len(total_times),
            "total_min_s": min(total_times),
            "total_max_s": max(total_times),
            "measurements": filtered,
        }
    
    def get_hot_paths(self) -> List[Dict[str, Any]]:
        """Identify hot paths (slowest phases)."""
        phase_times = {}
        
        for measurement in self.measurements:
            phases = measurement["timing"].get("phase_summary", {})
            for phase_name, elapsed in phases.items():
                if phase_name not in phase_times:
                    phase_times[phase_name] = []
                phase_times[phase_name].append(elapsed)
        
        # Calculate averages and identify slow phases
        hot_paths = []
        for phase_name, times in phase_times.items():
            avg_time = sum(times) / len(times)
            hot_paths.append({
                "phase": phase_name,
                "avg_time_s": avg_time,
                "count": len(times),
                "total_time_s": sum(times),
            })
        
        # Sort by total time (biggest impact)
        hot_paths.sort(key=lambda x: x["total_time_s"], reverse=True)
        
        return hot_paths
    
    def save_report(self, filepath: str):
        """Save performance report to file."""
        report = {
            "generated_at": time.time(),
            "total_measurements": len(self.measurements),
            "hot_paths": self.get_hot_paths(),
            "all_measurements": self.measurements,
        }
        
        with open(filepath, 'w') as f:
            json.dump(report, f, indent=2)
    
    def clear(self):
        """Clear all measurements."""
        self.measurements = []


# Global profiler instance
_profiler = PerformanceProfiler()


def get_profiler() -> PerformanceProfiler:
    """Get global performance profiler."""
    return _profiler


def record_measurement(command: str, timing: Dict[str, Any], metadata: Optional[Dict[str, Any]] = None):
    """Record a performance measurement to global profiler."""
    _profiler.record(command, timing, metadata)


# Example usage in ros2_cli.py:
"""
from ros2_telemetry import start_timing, phase, end_timing, record_measurement

def main(argv=None):
    # Parse args
    args = parse_args(argv)
    
    # Start timing
    timing = start_timing(f"{args.command} {args.subcommand or ''}")
    
    # Load config
    phase("load_config")
    config = load_config()
    
    # Execute command
    phase("execute")
    result = execute_command(args)
    
    # Finish timing
    phase("output")
    output(result)
    
    timing_data = end_timing()
    if timing_data:
        record_measurement(args.command, timing_data, {"returncode": 0})
"""
