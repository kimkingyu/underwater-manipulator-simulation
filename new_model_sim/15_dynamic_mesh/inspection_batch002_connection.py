"""One channel, bounded transport/health readiness, fixed read-only identity queries.

No API accepts user Scheme/TUI/Python. Both identity expressions below are copied
from installed official PyFluent readers. Readiness never starts another process,
channel or connection; only completed UNAVAILABLE health calls may be polled.
"""
from __future__ import annotations
import math
import re
import time

from mesh_inspection_connection import wait_private_credentials, safe_connection_error

FIXED_MODE_QUERY = "(cx-solver-mode?)"
FIXED_VERSION_QUERY = "(cx-version)"
TRANSPORT_READY_MAX_SECONDS = 20
HEALTH_CALL_MAX_SECONDS = 2
MAX_HEALTH_POLLS = 200


def normalize_loopback_alias(host):
    """Two ASCII literals only; never consult DNS, hosts, machine names or IP ranges."""
    if type(host) is not str or not host.isascii():
        raise ValueError("Private endpoint alias is not one of the two permitted ASCII literals")
    folded = host.casefold()
    if folded == "localhost":
        return "127.0.0.1", "literal_localhost"
    if folded == "127.0.0.1":
        return "127.0.0.1", "literal_ipv4_loopback"
    raise ValueError("Private endpoint alias is not one of the two permitted ASCII literals")


def readiness_passed(value):
    if not isinstance(value, dict):
        return False
    port = value.get("port")
    transport_limit = value.get("transport_wait_limit_seconds")
    finite_transport_limit = (type(transport_limit) in (int, float) and math.isfinite(transport_limit) and
                              0 < transport_limit <= TRANSPORT_READY_MAX_SECONDS)
    observations = value.get("health_observations")
    healthy_history = (isinstance(observations, list) and 1 <= len(observations) <= MAX_HEALTH_POLLS and
        observations[-1] == "SERVING" and all(x in ("SERVING", "NOT_SERVING", "UNKNOWN", "SERVICE_UNKNOWN", "RPC_UNAVAILABLE") for x in observations))
    rpc = value.get("health_rpc_observations")
    valid_rpc = (isinstance(rpc, list) and len(rpc) <= MAX_HEALTH_POLLS and all(isinstance(x, dict) and
        set(x) == {"poll", "rpc_status_codes"} and type(x["poll"]) is int and 1 <= x["poll"] <= MAX_HEALTH_POLLS and
        x["rpc_status_codes"] == ["UNAVAILABLE"] for x in rpc))
    valid_counts = (healthy_history and valid_rpc and type(value.get("health_poll_count")) is int and
        value["health_poll_count"] == len(observations) and type(value.get("health_unavailable_count")) is int and
        value["health_unavailable_count"] == observations.count("RPC_UNAVAILABLE") == len(rpc) and
        len({x["poll"] for x in rpc}) == len(rpc) and all(x["poll"] <= len(observations) and
            observations[x["poll"] - 1] == "RPC_UNAVAILABLE" for x in rpc))
    return (finite_transport_limit and valid_counts and value.get("provenance") == "live_fluent_batch002" and
        value.get("requested_address") == "127.0.0.1" and value.get("actual_endpoint_class") == "ipv4_loopback" and
        value.get("actual_address") == "127.0.0.1" and value.get("channel_target_address") == "127.0.0.1" and
        value.get("server_info_alias_class") in ("literal_localhost", "literal_ipv4_loopback") and
        value.get("alias_resolution") == "literal_only_no_dns" and type(port) is int and 1 <= port <= 65535 and
        value.get("transport_status") == "READY" and type(value.get("transport_wait_count")) is int and
        value["transport_wait_count"] == 1 and value.get("health_call_uncertain") is False and
        value.get("startup_window_within_deadline") is True and value.get("health_status") == "SERVING" and
        value.get("solver_mode_native") is True and value.get("native_version") == "24.2.0" and
        value.get("solver_session_constructed") is True and value.get("single_channel_count") == 1 and
        type(value.get("single_channel_count")) is int and value.get("connection_constructor_count") == 1 and
        type(value.get("connection_constructor_count")) is int and value.get("credentials_persisted") is False and
        value.get("scheme_identity_queries_completed") is True)


def connect_ready(attempt, process, deadline, evidence, *, dependencies=None, normal_exit=False):
    """Transport future and health share the caller's original 45-second deadline.

    Internal dependency injection is offline-only, never exposed by MCP/CLI.
    grpc.channel_ready_future(channel).result(timeout=min(20, remaining)) waits
    for transport READY on the single existing channel, not another connection.
    Only clearly completed health RPCs with a sole UNAVAILABLE status are polled
    again. Authentication rejection, unknown errors and timed-out/hung calls
    abort immediately; a hung call is never overlapped by another health call.
    """
    from definition_probe import bounded_call, UncertainCall
    evidence.update(provenance="live_fluent_batch002", requested_address="127.0.0.1",
        credentials_persisted=False, single_channel_count=0, connection_constructor_count=0,
        transport_wait_count=0, transport_status="unverified", health_status="unverified",
        health_poll_count=0, health_unavailable_count=0, health_observations=[], health_rpc_observations=[],
        health_call_uncertain=False, startup_window_within_deadline=False,
        solver_mode_native=False, solver_session_constructed=False)
    credentials = wait_private_credentials(attempt / "server.info", attempt, process, deadline, {"127.0.0.1", "localhost"})
    target, alias_class = normalize_loopback_alias(credentials.host)
    evidence.update(actual_address=target, actual_endpoint_class="ipv4_loopback", port=credentials.port,
        channel_target_address=target, server_info_alias_class=alias_class, alias_resolution="literal_only_no_dns")
    if dependencies is None:
        import grpc
        from ansys.fluent.core.fluent_connection import FluentConnection, ErrorState
        from ansys.fluent.core.services.health_check import HealthCheckService
        from ansys.fluent.core.session_solver import Solver
        dependencies = {"channel": grpc.insecure_channel, "ready_future": grpc.channel_ready_future,
                        "connection": FluentConnection, "health": HealthCheckService,
                        "error_state": ErrorState, "solver": Solver}
    remaining = lambda: max(0, deadline - time.monotonic())
    if remaining() <= 0:
        raise TimeoutError("No startup budget remains for a channel")
    channel = dependencies["channel"]("127.0.0.1:" + str(credentials.port), options=[
        ("grpc.max_send_message_length", 2147483647), ("grpc.max_receive_message_length", 2147483647)])
    evidence["single_channel_count"] = 1
    evidence["port_listener_evidence"] = "not_inferred_from_file; transport_and_authenticated_health_required"
    future = None
    try:
        future = dependencies["ready_future"](channel)
        evidence.update(transport_wait_count=1, transport_status="WAITING")
        transport_timeout = min(TRANSPORT_READY_MAX_SECONDS, remaining())
        if transport_timeout <= 0:
            raise TimeoutError("No startup budget remains for transport readiness")
        evidence["transport_wait_limit_seconds"] = transport_timeout
        try:
            bounded_call(lambda: future.result(timeout=transport_timeout), min(transport_timeout + 0.1, remaining()))
        except Exception:
            evidence["transport_status"] = "FAILED_OR_TIMED_OUT"
            raise
        evidence["transport_status"] = "READY"
        if remaining() <= 0:
            raise TimeoutError("Original startup deadline reached after transport readiness")
        health = dependencies["health"](channel, [("password", credentials.password)], dependencies["error_state"]())
        statuses, rpc_observations = [], []
        for poll in range(1, MAX_HEALTH_POLLS + 1):
            if remaining() <= 0:
                raise TimeoutError("Same-channel health did not become SERVING within startup budget")
            evidence["health_poll_count"] = poll
            try:
                status = bounded_call(health.check_health, min(HEALTH_CALL_MAX_SECONDS, remaining()))
            except UncertainCall:
                evidence["health_call_uncertain"] = True
                raise
            except Exception as exc:
                # bounded_call has returned this completed exception, so no
                # health RPC from this poll remains running before the next.
                codes = safe_connection_error(exc)["rpc_status_codes"]
                if codes != ["UNAVAILABLE"]:
                    evidence["health_failure_rpc_codes"] = codes
                    raise
                status = "RPC_UNAVAILABLE"
                rpc_observations.append({"poll": poll, "rpc_status_codes": ["UNAVAILABLE"]})
            if status not in ("SERVING", "NOT_SERVING", "UNKNOWN", "SERVICE_UNKNOWN", "RPC_UNAVAILABLE"):
                raise RuntimeError("Unexpected native health status")
            statuses.append(status)
            evidence.update(health_status=status, health_observations=list(statuses),
                health_rpc_observations=list(rpc_observations), health_unavailable_count=len(rpc_observations))
            if status == "SERVING":
                evidence["port_listener_evidence"] = "transport_READY_and_authenticated_health_SERVING_on_requested_loopback"
                break
            time.sleep(min(0.2, remaining()))
        else:
            raise TimeoutError("Fixed health poll limit reached without SERVING")
        if remaining() <= 0:
            raise TimeoutError("Original startup deadline reached before connection construction")
        # This public constructor reuses the same authenticated channel. The
        # cached/fallback Scheme version never qualifies as native identity.
        evidence["connection_constructor_count"] = 1
        connection = bounded_call(lambda: dependencies["connection"](channel=channel, password=credentials.password,
            cleanup_on_exit=normal_exit, start_transcript=False, inside_container=False), remaining())
        evidence.update(cleanup_on_exit=normal_exit, constructor_watchdog_process_started=False)
        del credentials
        mode = bounded_call(lambda: connection.scheme_eval.scheme_eval(FIXED_MODE_QUERY), remaining())
        evidence["solver_mode_native"] = mode is True
        if mode is not True:
            raise RuntimeError("Native authenticated mode query did not report solver")
        version = bounded_call(lambda: connection.scheme_eval.string_eval(FIXED_VERSION_QUERY), remaining())
        if not isinstance(version, str) or not re.fullmatch(r"\(\s*24\s+2\s+0\s*\)", version):
            raise RuntimeError("Native version query did not match fixed Fluent 24.2.0")
        evidence.update(native_version="24.2.0", scheme_identity_queries_completed=True)
        solver = bounded_call(lambda: dependencies["solver"](fluent_connection=connection), remaining())
        evidence["solver_session_constructed"] = True
        evidence["startup_window_within_deadline"] = remaining() > 0
        if not readiness_passed(evidence):
            raise RuntimeError("Native read-only readiness evidence incomplete or past deadline")
        return solver
    except BaseException:
        try:
            channel.close()
        except Exception:
            pass
        raise
    finally:
        if future is not None:
            # Cancels only the readiness subscription, not the existing channel
            # state machine or any process. No new wait or channel is created.
            try:
                future.cancel()
            except Exception:
                pass
