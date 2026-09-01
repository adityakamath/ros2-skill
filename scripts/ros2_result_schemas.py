#!/usr/bin/env python3
"""Standard result schemas and response envelopes for ros2-skill commands.

This module defines standard result formats to ensure consistent outputs
across all commands, making it easier for agents to parse and handle results.
"""

from enum import Enum
from typing import Any, Dict, List, Optional, Union


class ResultStatus(str, Enum):
    """Standard result status values."""
    SUCCESS = "success"
    FAILED = "failed"
    BLOCKED = "blocked"
    WARNED = "warned"
    TIMEOUT = "timeout"
    ERROR = "error"
    PARTIAL = "partial"


class ResultEnvelope:
    """Standard result envelope for all command outputs."""
    
    def __init__(
        self,
        status: ResultStatus,
        data: Optional[Dict[str, Any]] = None,
        error: Optional[str] = None,
        warning: Optional[str] = None,
        metadata: Optional[Dict[str, Any]] = None,
    ):
        self.status = status
        self.data = data or {}
        self.error = error
        self.warning = warning
        self.metadata = metadata or {}
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for JSON output."""
        result = {
            "status": self.status.value,
            "success": self.status == ResultStatus.SUCCESS,
        }
        
        if self.data:
            result.update(self.data)
        
        if self.error:
            result["error"] = self.error
        
        if self.warning:
            result["warning"] = self.warning
        
        if self.metadata:
            result["_metadata"] = self.metadata
        
        return result
    
    @classmethod
    def success(cls, data: Optional[Dict[str, Any]] = None, **kwargs) -> "ResultEnvelope":
        """Create a success result."""
        return cls(ResultStatus.SUCCESS, data=data, **kwargs)
    
    @classmethod
    def error(cls, message: str, **kwargs) -> "ResultEnvelope":
        """Create an error result."""
        return cls(ResultStatus.ERROR, error=message, **kwargs)
    
    @classmethod
    def blocked(cls, reason: str, data: Optional[Dict[str, Any]] = None, **kwargs) -> "ResultEnvelope":
        """Create a blocked result."""
        return cls(ResultStatus.BLOCKED, data=data, error=reason, **kwargs)
    
    @classmethod
    def warned(cls, message: str, data: Optional[Dict[str, Any]] = None, **kwargs) -> "ResultEnvelope":
        """Create a warned result."""
        return cls(ResultStatus.WARNED, data=data, warning=message, **kwargs)
    
    @classmethod
    def timeout(cls, message: str = "Operation timed out", **kwargs) -> "ResultEnvelope":
        """Create a timeout result."""
        return cls(ResultStatus.TIMEOUT, error=message, **kwargs)


class SafetyResult:
    """Standard result format for safety checks."""
    
    def __init__(
        self,
        allowed: bool,
        action: str,  # "allow", "block", "cap", "warn"
        violations: Optional[List[str]] = None,
        capped_data: Optional[Dict[str, Any]] = None,
        details: Optional[Dict[str, Any]] = None,
    ):
        self.allowed = allowed
        self.action = action
        self.violations = violations or []
        self.capped_data = capped_data
        self.details = details or {}
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary."""
        result = {
            "allowed": self.allowed,
            "action": self.action,
        }
        
        if self.violations:
            result["violations"] = self.violations
        
        if self.capped_data:
            result["capped_data"] = self.capped_data
        
        if self.details:
            result["details"] = self.details
        
        return result
    
    def to_envelope(self) -> ResultEnvelope:
        """Convert to standard result envelope."""
        if self.allowed:
            if self.action == "warn":
                return ResultEnvelope.warned(
                    f"Safety warning: {', '.join(self.violations)}",
                    data=self.to_dict()
                )
            else:
                return ResultEnvelope.success(data=self.to_dict())
        else:
            return ResultEnvelope.blocked(
                f"Safety violation: {', '.join(self.violations)}",
                data=self.to_dict()
            )


class MissionResult:
    """Standard result format for mission execution."""
    
    def __init__(
        self,
        mission_id: str,
        status: str,  # "pending", "running", "paused", "completed", "failed", "canceled"
        current_step: Optional[int] = None,
        total_steps: Optional[int] = None,
        history: Optional[List[Dict[str, Any]]] = None,
        failure_reason: Optional[str] = None,
    ):
        self.mission_id = mission_id
        self.status = status
        self.current_step = current_step
        self.total_steps = total_steps
        self.history = history or []
        self.failure_reason = failure_reason
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary."""
        result = {
            "mission_id": self.mission_id,
            "status": self.status,
        }
        
        if self.current_step is not None:
            result["current_step"] = self.current_step
        
        if self.total_steps is not None:
            result["total_steps"] = self.total_steps
            result["progress_pct"] = (self.current_step / self.total_steps * 100) if self.total_steps > 0 else 0
        
        if self.history:
            result["history"] = self.history
            result["completed_steps"] = sum(1 for h in self.history if h.get("ok"))
        
        if self.failure_reason:
            result["failure_reason"] = self.failure_reason
        
        return result
    
    def to_envelope(self) -> ResultEnvelope:
        """Convert to standard result envelope."""
        data = self.to_dict()
        
        if self.status == "completed":
            return ResultEnvelope.success(data=data)
        elif self.status == "failed":
            return ResultEnvelope.error(self.failure_reason or "Mission failed", data=data)
        elif self.status == "canceled":
            return ResultEnvelope(ResultStatus.PARTIAL, data=data, warning="Mission canceled")
        else:
            return ResultEnvelope.success(data=data)


class TimingMetadata:
    """Standard timing metadata for command execution."""
    
    def __init__(
        self,
        total_s: float,
        phases: Optional[Dict[str, float]] = None,
        started_at: Optional[float] = None,
        completed_at: Optional[float] = None,
    ):
        self.total_s = total_s
        self.phases = phases or {}
        self.started_at = started_at
        self.completed_at = completed_at
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary."""
        result = {"total_s": self.total_s}
        
        if self.phases:
            result["phases"] = self.phases
        
        if self.started_at:
            result["started_at"] = self.started_at
        
        if self.completed_at:
            result["completed_at"] = self.completed_at
        
        return result


def standardize_result(
    success: bool,
    data: Optional[Dict[str, Any]] = None,
    error: Optional[str] = None,
    warning: Optional[str] = None,
    blocked: bool = False,
    timing: Optional[TimingMetadata] = None,
) -> Dict[str, Any]:
    """
    Standardize a command result into consistent format.
    
    This is a convenience function for converting existing command outputs
    to the standard envelope format.
    """
    if blocked:
        envelope = ResultEnvelope.blocked(error or "Operation blocked", data=data)
    elif not success:
        envelope = ResultEnvelope.error(error or "Operation failed", data=data)
    elif warning:
        envelope = ResultEnvelope.warned(warning, data=data)
    else:
        envelope = ResultEnvelope.success(data=data)
    
    result = envelope.to_dict()
    
    if timing:
        result["_metadata"]["timing"] = timing.to_dict()
    
    return result


# Migration helpers for existing code
def wrap_legacy_result(legacy_output: Dict[str, Any]) -> Dict[str, Any]:
    """
    Wrap a legacy command output in standard envelope format.
    
    Detects common patterns in existing outputs and converts them
    to standard format.
    """
    # Check for existing status indicators
    if "error" in legacy_output:
        return ResultEnvelope.error(
            legacy_output["error"],
            data={k: v for k, v in legacy_output.items() if k != "error"}
        ).to_dict()
    
    if legacy_output.get("blocked"):
        return ResultEnvelope.blocked(
            legacy_output.get("blocked"),
            data={k: v for k, v in legacy_output.items() if k != "blocked"}
        ).to_dict()
    
    if "warning" in legacy_output:
        return ResultEnvelope.warned(
            legacy_output["warning"],
            data={k: v for k, v in legacy_output.items() if k != "warning"}
        ).to_dict()
    
    # Check for success field
    if "success" in legacy_output:
        if legacy_output["success"]:
            return ResultEnvelope.success(data=legacy_output).to_dict()
        else:
            return ResultEnvelope.error(
                legacy_output.get("error", "Operation failed"),
                data=legacy_output
            ).to_dict()
    
    # Default: assume success
    return ResultEnvelope.success(data=legacy_output).to_dict()


# Example usage patterns:
"""
# In command handlers:

def cmd_topics_list(args):
    try:
        topics = get_topics()
        return ResultEnvelope.success(data={"topics": topics}).to_dict()
    except Exception as e:
        return ResultEnvelope.error(str(e)).to_dict()

def cmd_safety_check(args):
    allowed, reason, details = check_velocity(msg, config)
    safety_result = SafetyResult(allowed, "block" if not allowed else "allow", [reason], details=details)
    return safety_result.to_envelope().to_dict()

def cmd_mission_status(args):
    state = load_mission_state(args.mission_id)
    mission_result = MissionResult(
        mission_id=state["mission_id"],
        status=state["status"],
        current_step=state.get("current_step"),
        total_steps=len(state.get("steps", [])),
        history=state.get("history")
    )
    return mission_result.to_envelope().to_dict()
"""
